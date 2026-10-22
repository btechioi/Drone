#!/usr/bin/env python3
"""
test_protocol.py - self test for the drone link protocol.

Verifies that the host side (tools/gcs.py) agrees with the firmware side
(firmware/src/protocol.h, firmware/src/telemetry.cpp, and madflight's
tbx_crc16()).

Run with:  python3 tools/test_protocol.py
"""

import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import gcs  # noqa: E402

FAILURES = []


def check(name, got, expected):
    if got == expected:
        print("  ok   %s" % name)
    else:
        print("  FAIL %s: got %r, expected %r" % (name, got, expected))
        FAILURES.append(name)


def checkf(name, got, expected):
    """Compare floats with a tolerance (the wire format is float32)."""
    check(name, abs(got - expected) < 1e-6, True)


def test_crc_vectors():
    """CRC16-CCITT-FALSE vectors, generated with madflight's tbx_crc16()."""
    print("crc16 (verified against madflight src/tbx/tbx_crc.cpp)")
    check("crc of 01 08 00", gcs.crc16(bytes([0x01, 0x08, 0x00])), 29189)
    check("crc of 02 10 00", gcs.crc16(bytes([0x02, 0x10, 0x00])), 41359)
    check(
        "crc of 03 00 01..05",
        gcs.crc16(bytes([0x03, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05])),
        43844,
    )
    buf = bytes(((i * 7 + 3) & 0xFF) for i in range(64))
    check("crc of 64 byte pattern", gcs.crc16(buf), 32868)


def test_frame_layout():
    """The frame layout must be: A5 5A | id | len(2 LE) | payload | crc(2 LE)."""
    print("frame layout")
    frame = gcs.build_frame(gcs.CMD_PING)
    check("ping frame length", len(frame), 7)
    check("ping sync", frame[0:2], bytes([0xA5, 0x5A]))
    check("ping id", frame[2], gcs.CMD_PING)
    check("ping length", frame[3:5], struct.pack("<H", 0))
    check(
        "ping crc",
        frame[5:7],
        struct.pack("<H", gcs.crc16(bytes([gcs.CMD_PING, 0x00, 0x00]))),
    )


def test_roundtrip():
    """Every control frame must survive the parser unchanged."""
    print("control frame round trip")
    parser = gcs.FrameParser()

    frames = [
        gcs.frame_ping(),
        gcs.frame_set_channels([1500, 1500, 1100, 1500, 1000, 1165]),
        gcs.frame_set_sticks(0.25, -0.5, 0.75, 0.0, False, 3),
        gcs.frame_set_sticks(0.0, 0.0, 0.0, 0.0, True, 0),
    ]

    for frame in frames:
        out = parser.feed(frame)
        check("one frame decoded", len(out), 1)
        msg_id, payload = out[0]
        check("id of 0x%02X" % frame[2], msg_id, frame[2])
        check(
            "payload of 0x%02X" % frame[2],
            payload,
            frame[5 : 5 + struct.unpack("<H", frame[3:5])[0]],
        )

    # feed everything at once, plus some garbage in front
    blob = b"\x00\xff\xa5" + b"".join(frames) + b"\x11"
    out = parser.feed(blob)
    check("4 frames decoded from a blob", len(out), 4)

    # a corrupted crc must be rejected
    bad = bytearray(frames[1])
    bad[-1] ^= 0xFF
    parser2 = gcs.FrameParser()
    out = parser2.feed(bytes(bad))
    check("corrupted frame rejected", len(out), 0)
    check("corrupted frame counted", parser2.errors, 1)


def test_telemetry_decode():
    """Build telemetry payloads exactly like telemetry.cpp does and decode them."""
    print("telemetry decoding")

    # STATUS: u32 uptime, u8 flags, u8 mode, u16 rate, 5x u32
    p = struct.pack(
        "<IBBHIIIII",
        123456,
        gcs.FLAG_ARMED | gcs.FLAG_IMU_OK,
        2,
        1000,
        5000,
        0,
        120,
        900,
        3,
    )
    check("status payload size", len(p), 28)
    d = gcs.decode_status(p)
    check("status uptime", d["uptime_ms"], 123456)
    check("status flags", d["flags"], 0x05)
    check("status imu rate", d["imu_rate"], 1000)
    check("status frames dropped", d["frames_dropped"], 3)

    # IMU: 7 floats
    p = struct.pack("<7f", 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)
    d = gcs.decode_imu(p)
    checkf("imu ax", d["ax"], 1.0)
    checkf("imu gz", d["gz"], 6.0)
    checkf("imu temp", d["temp"], 7.0)

    # ATTITUDE: 7 floats
    p = struct.pack("<7f", 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)
    d = gcs.decode_attitude(p)
    checkf("attitude roll", d["roll"], 1.0)
    checkf("attitude qz", d["qz"], 7.0)

    # ALTITUDE: 6 floats
    p = struct.pack("<6f", 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    d = gcs.decode_altitude(p)
    checkf("altitude alt", d["alt"], 3.0)
    checkf("altitude press", d["press"], 5.0)

    # MAG: 3 floats
    p = struct.pack("<3f", 10.0, 20.0, 30.0)
    d = gcs.decode_mag(p)
    checkf("mag mx", d["mx"], 10.0)
    checkf("mag mz", d["mz"], 30.0)

    # RC: 5 floats, 4 bytes, 8 x u16
    p = struct.pack("<5f4B8H", 0.1, 0.2, 0.3, 0.4, 0.5, 1, 2, 0, 0, *range(1000, 1008))
    check("rc payload size", len(p), 40)
    d = gcs.decode_rc(p)
    checkf("rc roll", d["roll"], 0.1)
    check("rc armed", d["armed"], 1)
    check("rc flightmode", d["flightmode"], 2)
    check("rc pwm[0]", d["pwm"][0], 1000)
    check("rc pwm[7]", d["pwm"][7], 1007)

    # BATTERY: 5 floats
    p = struct.pack("<5f", 12.0, 1.0, 12.0, 100.0, 1.2)
    d = gcs.decode_battery(p)
    checkf("battery v", d["v"], 12.0)
    checkf("battery mah", d["mah"], 100.0)

    # MOTORS: 4 bytes, 8 floats
    p = struct.pack("<4B8f", 1, 1, 4, 0, 0.1, 0.2, 0.3, 0.4, 0, 0, 0, 0)
    check("motors payload size", len(p), 36)
    d = gcs.decode_motors(p)
    check("motors armed", d["armed"], 1)
    checkf("motors[0]", d["motors"][0], 0.1)
    checkf("motors[3]", d["motors"][3], 0.4)

    # PID: 12 floats, 3 per axis (roll, pitch, yaw, throttle)
    values = [float(i) for i in range(12)]
    p = struct.pack("<12f", *values)
    check("pid payload size", len(p), 48)
    d = gcs.decode_pid(p)
    checkf("pid roll sp", d["roll"]["sp"], 0.0)
    checkf("pid roll sum", d["roll"]["sum"], 2.0)
    checkf("pid pitch sp", d["pitch"]["sp"], 3.0)
    checkf("pid yaw actual", d["yaw"]["actual"], 7.0)
    checkf("pid throttle sp", d["throttle"]["sp"], 9.0)
    checkf("pid throttle actual", d["throttle"]["actual"], 10.0)
    checkf("pid throttle sum", d["throttle"]["sum"], 11.0)

    # EVENT: id, len, text
    p = bytes([gcs.EVENT_ARMED, 5]) + b"ARMED"
    d = gcs.decode_event(p)
    check("event id", d["event"], gcs.EVENT_ARMED)
    check("event text", d["text"], "ARMED")


def test_all_telemetry_ids_decode():
    """Every telemetry message id must have a decoder."""
    print("telemetry ids")
    for msg_id in range(1, 11):
        check("decoder for 0x%02X" % msg_id, msg_id in gcs.DECODERS, True)


def main():
    test_crc_vectors()
    test_frame_layout()
    test_roundtrip()
    test_telemetry_decode()
    test_all_telemetry_ids_decode()
    print()
    if FAILURES:
        print("%d FAILURES: %s" % (len(FAILURES), ", ".join(FAILURES)))
        return 1
    print("all tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
