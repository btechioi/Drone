#!/usr/bin/env python3
"""
gcs.py - Ground station tool for the Pico 2 / GY-87 drone firmware.

Speaks the binary link protocol described in firmware/src/protocol.h:

    offset  size  field
    0       1     0xA5          sync byte 1
    1       1     0x5A          sync byte 2
    2       1     message id
    3       2     payload length in bytes (little endian)
    5       N     payload
    5+N     2     CRC16-CCITT (init 0xFFFF, poly 0x1021) over bytes [2 .. 4+N]

Two independent links are used, each on its own serial port:

    --telemetry PORT   flight controller -> host   (UART1, GPIO8 = TX)
    --control   PORT   host -> flight controller   (UART0, GPIO1 = RX)

Examples
--------

Watch the telemetry:

    python3 tools/gcs.py --telemetry /dev/ttyUSB0

Send normalized sticks (roll, pitch, yaw, throttle) at 20 Hz for 10 seconds:

    python3 tools/gcs.py --control /dev/ttyUSB1 --sticks 0 0 0 0 --duration 10

Send raw channel PWM values (AETR + arm + flightmode):

    python3 tools/gcs.py --control /dev/ttyUSB1 --channels 1500 1500 1100 1500 1000 1165

Arm the drone (throttle must be at zero):

    python3 tools/gcs.py --control /dev/ttyUSB1 --sticks 0 0 0 0 --arm --duration 5

Keep the control link alive without sending any stick data:

    python3 tools/gcs.py --control /dev/ttyUSB1 --ping

Requires pyserial:  pip install pyserial
"""

import argparse
import math
import struct
import sys
import time

try:
    import serial
except ImportError:  # pragma: no cover - depends on the host
    sys.exit("This tool needs pyserial. Install it with: pip install pyserial")


# ----------------------------------------------------------------------------
# Protocol constants (must match firmware/src/protocol.h)
# ----------------------------------------------------------------------------

SYNC1 = 0xA5
SYNC2 = 0x5A
MAX_PAYLOAD = 160

# telemetry message ids (flight controller -> host)
MSG_STATUS = 0x01
MSG_IMU = 0x02
MSG_ATTITUDE = 0x03
MSG_ALTITUDE = 0x04
MSG_MAG = 0x05
MSG_RC = 0x06
MSG_BATTERY = 0x07
MSG_MOTORS = 0x08
MSG_PID = 0x09
MSG_EVENT = 0x0A

# control message ids (host -> flight controller)
CMD_SET_CHANNELS = 0x01
CMD_SET_STICKS = 0x02
CMD_PING = 0x03

# status flags
FLAG_ARMED = 0x01
FLAG_RCL_LINK = 0x02
FLAG_IMU_OK = 0x04
FLAG_BAR_OK = 0x08
FLAG_MAG_OK = 0x10
FLAG_BAT_OK = 0x20
FLAG_RCL_FAULT = 0x40

# event ids
EVENT_TEXT = 0x01
EVENT_ARMED = 0x02
EVENT_DISARMED = 0x03
EVENT_ERROR = 0x04


# ----------------------------------------------------------------------------
# Framing
# ----------------------------------------------------------------------------


def crc16(data, crc=0xFFFF):
    """CRC16-CCITT-FALSE, the same function as madflight's tbx_crc16()."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(msg_id, payload=b""):
    """Build a complete protocol frame (sync, id, length, payload, crc)."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long: %d > %d" % (len(payload), MAX_PAYLOAD))
    body = struct.pack("<BH", msg_id, len(payload)) + payload
    return bytes([SYNC1, SYNC2]) + body + struct.pack("<H", crc16(body))


class FrameParser:
    """Incremental frame parser - feed it bytes, get back (id, payload) tuples."""

    def __init__(self):
        self.buf = bytearray()
        self.errors = 0

    def feed(self, data):
        """Feed bytes, return a list of decoded (msg_id, payload) frames."""
        self.buf += data
        frames = []
        while True:
            # find the sync bytes
            start = self.buf.find(bytes([SYNC1, SYNC2]))
            if start < 0:
                # no sync: keep a trailing 0xA5 in case it is a partial sync
                if self.buf and self.buf[-1] == SYNC1:
                    del self.buf[:-1]
                else:
                    del self.buf[:]
                break
            if start > 0:
                del self.buf[:start]
            if len(self.buf) < 5:
                break
            msg_id = self.buf[2]
            length = self.buf[3] | (self.buf[4] << 8)
            if length > MAX_PAYLOAD:
                # corrupt frame, resync
                del self.buf[:1]
                self.errors += 1
                continue
            total = 5 + length + 2
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2 : 5 + length])
            crc_rx = self.buf[5 + length] | (self.buf[6 + length] << 8)
            del self.buf[:total]
            if crc16(body) != crc_rx:
                self.errors += 1
                continue
            frames.append((msg_id, bytes(body[3:])))
        return frames


# ----------------------------------------------------------------------------
# Telemetry decoding
# ----------------------------------------------------------------------------


def _f32(payload, offset):
    return struct.unpack_from("<f", payload, offset)[0]


def _u32(payload, offset):
    return struct.unpack_from("<I", payload, offset)[0]


def _u16(payload, offset):
    return struct.unpack_from("<H", payload, offset)[0]


def _u8(payload, offset):
    return payload[offset]


def decode_status(p):
    flags = _u8(p, 4)
    names = []
    for bit, name in (
        (FLAG_ARMED, "ARMED"),
        (FLAG_RCL_LINK, "RCL_LINK"),
        (FLAG_IMU_OK, "IMU_OK"),
        (FLAG_BAR_OK, "BAR_OK"),
        (FLAG_MAG_OK, "MAG_OK"),
        (FLAG_BAT_OK, "BAT_OK"),
        (FLAG_RCL_FAULT, "RCL_FAULT"),
    ):
        if flags & bit:
            names.append(name)
    return {
        "uptime_ms": _u32(p, 0),
        "flags": flags,
        "flag_names": "|".join(names) if names else "-",
        "flightmode": _u8(p, 5),
        "imu_rate": _u16(p, 6),
        "imu_updates": _u32(p, 8),
        "imu_overruns": _u32(p, 12),
        "rcl_updates": _u32(p, 16),
        "frames_sent": _u32(p, 20),
        "frames_dropped": _u32(p, 24),
    }


def decode_imu(p):
    return {
        "ax": _f32(p, 0),
        "ay": _f32(p, 4),
        "az": _f32(p, 8),
        "gx": _f32(p, 12),
        "gy": _f32(p, 16),
        "gz": _f32(p, 20),
        "temp": _f32(p, 24),
    }


def decode_attitude(p):
    return {
        "roll": _f32(p, 0),
        "pitch": _f32(p, 4),
        "yaw": _f32(p, 8),
        "qw": _f32(p, 12),
        "qx": _f32(p, 16),
        "qy": _f32(p, 20),
        "qz": _f32(p, 24),
    }


def decode_altitude(p):
    return {
        "bar_alt": _f32(p, 0),
        "ground_level": _f32(p, 4),
        "alt": _f32(p, 8),
        "vspeed": _f32(p, 12),
        "press": _f32(p, 16),
        "temp": _f32(p, 20),
    }


def decode_mag(p):
    return {"mx": _f32(p, 0), "my": _f32(p, 4), "mz": _f32(p, 8)}


def decode_rc(p):
    pwms = [_u16(p, 24 + 2 * i) for i in range(8)]
    return {
        "roll": _f32(p, 0),
        "pitch": _f32(p, 4),
        "yaw": _f32(p, 8),
        "throttle": _f32(p, 12),
        "vspeed": _f32(p, 16),
        "armed": _u8(p, 20),
        "flightmode": _u8(p, 21),
        "pwm": pwms,
    }


def decode_battery(p):
    return {
        "v": _f32(p, 0),
        "i": _f32(p, 4),
        "w": _f32(p, 8),
        "mah": _f32(p, 12),
        "wh": _f32(p, 16),
    }


def decode_motors(p):
    return {
        "armed": _u8(p, 0),
        "mode": _u8(p, 1),
        "motors": [_f32(p, 4 + 4 * i) for i in range(8)],
    }


def decode_pid(p):
    out = {}
    for i, axis in enumerate(("roll", "pitch", "yaw", "throttle")):
        base = i * 12
        out[axis] = {
            "sp": _f32(p, base),
            "actual": _f32(p, base + 4),
            "sum": _f32(p, base + 8),
        }
    return out


def decode_event(p):
    event_id = _u8(p, 0)
    length = _u8(p, 1)
    text = p[2 : 2 + length].decode("ascii", "replace")
    return {"event": event_id, "text": text}


DECODERS = {
    MSG_STATUS: ("STATUS", decode_status),
    MSG_IMU: ("IMU", decode_imu),
    MSG_ATTITUDE: ("ATTITUDE", decode_attitude),
    MSG_ALTITUDE: ("ALTITUDE", decode_altitude),
    MSG_MAG: ("MAG", decode_mag),
    MSG_RC: ("RC", decode_rc),
    MSG_BATTERY: ("BATTERY", decode_battery),
    MSG_MOTORS: ("MOTORS", decode_motors),
    MSG_PID: ("PID", decode_pid),
    MSG_EVENT: ("EVENT", decode_event),
}


def format_decoded(msg_id, payload):
    entry = DECODERS.get(msg_id)
    if entry is None:
        return "UNKNOWN id=0x%02X len=%d %s" % (msg_id, len(payload), payload.hex())
    name, decoder = entry
    try:
        data = decoder(payload)
    except (struct.error, IndexError) as exc:
        return "%s decode error: %s (%d bytes)" % (name, exc, len(payload))
    if msg_id == MSG_EVENT:
        return "EVENT   [%d] %s" % (data["event"], data["text"])
    if msg_id == MSG_PID:
        parts = [
            "%s sp=%+.3f act=%+.3f sum=%+.3f" % (axis, v["sp"], v["actual"], v["sum"])
            for axis, v in data.items()
        ]
        return "PID      " + "  ".join(parts)
    if msg_id == MSG_MOTORS:
        motors = " ".join("%d:%.3f" % (i + 1, v) for i, v in enumerate(data["motors"][:4]))
        return "MOTORS   armed=%d mode=%d  %s" % (data["armed"], data["mode"], motors)
    if msg_id == MSG_RC:
        pwms = " ".join(str(v) for v in data["pwm"][:6])
        return (
            "RC       roll=%+.3f pitch=%+.3f yaw=%+.3f thr=%+.3f vs=%+.3f armed=%d mode=%d pwm=[%s]"
            % (
                data["roll"],
                data["pitch"],
                data["yaw"],
                data["throttle"],
                data["vspeed"],
                data["armed"],
                data["flightmode"],
                pwms,
            )
        )
    if msg_id == MSG_STATUS:
        return (
            "STATUS   up=%.1fs flags=%s mode=%d imu_rate=%d imu_upd=%d over=%d rcl_upd=%d sent=%d drop=%d"
            % (
                data["uptime_ms"] / 1000.0,
                data["flag_names"],
                data["flightmode"],
                data["imu_rate"],
                data["imu_updates"],
                data["imu_overruns"],
                data["rcl_updates"],
                data["frames_sent"],
                data["frames_dropped"],
            )
        )
    if msg_id == MSG_IMU:
        return (
            "IMU      a=(%+.3f,%+.3f,%+.3f)G g=(%+.1f,%+.1f,%+.1f)d/s t=%.1fC"
            % (
                data["ax"],
                data["ay"],
                data["az"],
                data["gx"],
                data["gy"],
                data["gz"],
                data["temp"],
            )
        )
    if msg_id == MSG_ATTITUDE:
        return "ATTITUDE roll=%+.2f pitch=%+.2f yaw=%+.2f" % (
            data["roll"],
            data["pitch"],
            data["yaw"],
        )
    if msg_id == MSG_ALTITUDE:
        return "ALTITUDE alt=%+.2fm v=%+.2fm/s bar=%+.2fm press=%.1fPa t=%.1fC" % (
            data["alt"],
            data["vspeed"],
            data["bar_alt"],
            data["press"],
            data["temp"],
        )
    if msg_id == MSG_MAG:
        return "MAG      n=%+.1f e=%+.1f d=%+.1f uT" % (data["mx"], data["my"], data["mz"])
    if msg_id == MSG_BATTERY:
        return "BATTERY  %.2fV %.2fA %.1fW %.0fmAh %.2fWh" % (
            data["v"],
            data["i"],
            data["w"],
            data["mah"],
            data["wh"],
        )
    return "%s %s" % (name, data)


# ----------------------------------------------------------------------------
# Control frames (host -> flight controller)
# ----------------------------------------------------------------------------


def frame_set_sticks(roll, pitch, yaw, throttle, arm=False, flightmode=0):
    """Build a SET_STICKS frame.

    roll      -1.0 (left)  .. +1.0 (right)
    pitch     -1.0 (stick forward) .. +1.0 (stick back)
    yaw       -1.0 (left)  .. +1.0 (right)
    throttle  0.0 (idle)   .. +1.0 (full)
    arm        True to request arming (throttle must be zero)
    flightmode 0..5
    """
    payload = struct.pack(
        "<ffffBB", roll, pitch, yaw, throttle, 1 if arm else 0, flightmode & 0xFF
    )
    return build_frame(CMD_SET_STICKS, payload)


def frame_set_channels(channels):
    """Build a SET_CHANNELS frame from a list of PWM values in microseconds."""
    channels = [int(c) for c in channels][:20]
    payload = struct.pack("<%dH" % len(channels), *channels)
    return build_frame(CMD_SET_CHANNELS, payload)


def frame_ping():
    return build_frame(CMD_PING)


# ----------------------------------------------------------------------------
# Serial helpers
# ----------------------------------------------------------------------------


def open_port(port, baud):
    try:
        return serial.Serial(port, baud, timeout=0.05)
    except serial.SerialException as exc:
        sys.exit("Cannot open %s: %s" % (port, exc))


def run_telemetry(port, baud, duration, raw):
    """Read and print telemetry frames until interrupted."""
    ser = open_port(port, baud)
    parser = FrameParser()
    print("Telemetry: reading %s at %d baud (Ctrl-C to stop)" % (port, baud))
    start = time.time()
    frames = 0
    try:
        while duration <= 0 or (time.time() - start) < duration:
            chunk = ser.read(256)
            if chunk:
                for msg_id, payload in parser.feed(chunk):
                    frames += 1
                    if raw:
                        print("0x%02X %s" % (msg_id, payload.hex()))
                    else:
                        print(format_decoded(msg_id, payload))
            if parser.errors:
                sys.stderr.write("framing errors: %d\n" % parser.errors)
                parser.errors = 0
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
    print("--- %d frames received, %d framing errors ---" % (frames, parser.errors))


def run_control(port, baud, rate, duration, frame_fn):
    """Send control frames at a fixed rate until interrupted."""
    ser = open_port(port, baud)
    period = 1.0 / rate if rate > 0 else 0.05
    print(
        "Control: writing %s at %d baud, %d Hz (Ctrl-C to stop)"
        % (port, baud, rate)
    )
    sent = 0
    start = time.time()
    next_tx = start
    try:
        while duration <= 0 or (time.time() - start) < duration:
            now = time.time()
            if now < next_tx:
                time.sleep(min(next_tx - now, 0.01))
                continue
            next_tx = now + period
            ser.write(frame_fn())
            sent += 1
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
    print("--- %d control frames sent ---" % sent)


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description="Ground station tool for the Pico 2 / GY-87 drone firmware",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--telemetry", metavar="PORT", help="serial port with flight controller telemetry"
    )
    parser.add_argument(
        "--control", metavar="PORT", help="serial port to send control frames to"
    )
    parser.add_argument("--baud", type=int, default=115200, help="baud rate (default 115200)")
    parser.add_argument(
        "--duration",
        type=float,
        default=0,
        help="stop after this many seconds (0 = run until Ctrl-C)",
    )
    parser.add_argument(
        "--raw", action="store_true", help="print raw telemetry payloads as hex"
    )
    parser.add_argument(
        "--rate", type=int, default=20, help="control frame rate in Hz (default 20)"
    )

    group = parser.add_argument_group("control data")
    group.add_argument(
        "--sticks",
        type=float,
        nargs=4,
        metavar=("ROLL", "PITCH", "YAW", "THROTTLE"),
        help="send normalized stick values (-1..1, throttle 0..1)",
    )
    group.add_argument(
        "--channels",
        type=int,
        nargs="+",
        metavar="PWM",
        help="send raw channel PWM values in us (ch1=roll ch2=pitch ch3=throttle "
        "ch4=yaw ch5=arm ch6=flightmode)",
    )
    group.add_argument(
        "--ping", action="store_true", help="only send keepalive frames"
    )
    group.add_argument(
        "--arm", action="store_true", help="request arming (only with --sticks)"
    )
    group.add_argument(
        "--flightmode", type=int, default=0, help="flight mode index 0..5 (default 0)"
    )

    args = parser.parse_args()

    if not args.telemetry and not args.control:
        parser.print_help()
        return 1

    if args.sticks and args.channels:
        sys.exit("Use either --sticks or --channels, not both.")

    # --- control link ---
    if args.control:
        if args.sticks:
            roll, pitch, yaw, throttle = args.sticks
            if args.arm and throttle != 0:
                sys.exit(
                    "Refusing to arm with a non-zero throttle "
                    "(the flight controller requires throttle zero to arm)."
                )
            frame_fn = lambda: frame_set_sticks(  # noqa: E731
                roll, pitch, yaw, throttle, args.arm, args.flightmode
            )
            print(
                "WARNING: this sends stick data to the flight controller. "
                "Make sure the props are clear."
            )
        elif args.channels:
            frame_fn = lambda: frame_set_channels(args.channels)  # noqa: E731
        elif args.ping:
            frame_fn = frame_ping
        else:
            sys.exit(
                "Nothing to send on the control link. "
                "Use --sticks, --channels or --ping."
            )
        run_control(args.control, args.baud, args.rate, args.duration, frame_fn)

    # --- telemetry link ---
    if args.telemetry:
        run_telemetry(args.telemetry, args.baud, args.duration, args.raw)

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(0)
