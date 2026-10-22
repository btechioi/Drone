# Drone firmware - Raspberry Pi Pico 2 (RP2350) + GY-87 10DOF

Flight controller firmware built on the [madflight](https://madflight.com) flight
controller library.

* **MCU** &mdash; Raspberry Pi Pico 2 (RP2350A, 30 GPIO)
* **Sensors** &mdash; GY-87 10DOF board (MPU6050 IMU + BMP280 barometer + HMC5883L magnetometer)
* **Control input** &mdash; binary RC protocol over UART0
* **Telemetry output** &mdash; binary flight data over UART1
* **Console** &mdash; madflight CLI over USB-CDC (115200 baud)

---

## 1. Hardware

### 1.1 Parts list

| Part | Notes |
|---|---|
| Raspberry Pi Pico 2 | RP2350A, 30 GPIO, sold separately |
| GY-87 10DOF | MPU6050 + BMP280 + HMC5883L on one I2C bus |
| 4x ESC + motor | brushless (standard PWM) or brushed |
| USB cable | for the console and for flashing |
| 2x USB-serial adapter (optional) | one for the control link, one for telemetry |

### 1.2 Wiring

All GPIO numbers are Pico 2 **GPIO** numbers, not physical pin numbers.

| Function | Pico 2 GPIO | Direction | Notes |
|---|---|---|---|
| GY-87 SDA | 20 | bidirectional | I2C0 SDA |
| GY-87 SCL | 21 | bidirectional | I2C0 SCL |
| GY-87 MPU6050 INT | 22 | input | **required** - madflight panics if the IMU interrupt never fires |
| RC control input | 1 (UART0 RX) | input | host transmits, flight controller receives |
| RC control output | 0 (UART0 TX) | output | not used, but keep it connected for a shared ground/loopback |
| Telemetry output | 8 (UART1 TX) | output | flight controller transmits |
| Telemetry input | 9 (UART1 RX) | input | not used by the firmware |
| Motor 1 | 2 | output | ESC signal, PWM slice 1 |
| Motor 2 | 3 | output | ESC signal, PWM slice 1 |
| Motor 3 | 4 | output | ESC signal, PWM slice 2 |
| Motor 4 | 5 | output | ESC signal, PWM slice 2 |
| Status LED | 25 | output | on-board LED |
| Console | USB-CDC | - | native USB, no UART needed |
| Battery sense (optional) | 27 / 28 | input | ADC1 = current, ADC2 = voltage (see section 8) |

**Common ground:** the Pico 2, the GY-87, the ESCs and the USB-serial adapters
must all share a common ground.

**Note on the motors:** GPIO2..GPIO5 are the only pins that map cleanly onto the
PWM slices, which is why the four motors are there and the UARTs are on GPIO0/1
and GPIO8/9.

### 1.3 GY-87 I2C addresses

All three chips sit on the same I2C bus:

| Chip | Address | Address in the config file |
|---|---|---|
| MPU6050 | 0x68 | `imu_i2c_adr 104` |
| BMP280 | 0x76 | `bar_i2c_adr 118` |
| HMC5883L | 0x1E | `mag_i2c_adr 30` |

The config file uses **decimal** numbers only. Run `i2c` on the CLI to verify
that all three devices answer.

---

## 2. Building and flashing

Requirements: [PlatformIO Core](https://platformio.org/) (the `pio` command).

```bash
cd firmware
pio run -e RP2350A                 # build
pio run -e RP2350A -t upload       # build and flash over USB
pio device monitor -e RP2350A      # open the console
```

The build regenerates madflight's parameter header (`src/cfg/param.yaml.h`)
from `src/cfg/param.yaml` on every run, so the C enums always match the
parameter definitions. The generator lives in `tools/gen_param.py` and is
invoked by the PlatformIO `pre:` script `tools/pre_gen_param.py`.

The madflight library is not vendored into this project, it is referenced as a
symlinked library:

```ini
lib_deps =
    symlink://../madflight
```

So the repository layout is:

```
Drone/
├── madflight/          <- the madflight library (git submodule)
└── firmware/           <- this project
```

`madflight/` is a git submodule pointing at the fork
[btechioi/madflight](https://github.com/btechioi/madflight), branch
`feature/pico2-gy87`, which contains the library changes listed in
section 9.1. Clone with the submodule:

```bash
git clone --recurse-submodules https://github.com/btechioi/Drone.git
```

or, in an existing clone:

```bash
git submodule update --init --recursive
```

The same library changes are also submitted upstream as
[PR #168](https://github.com/qqqlab/madflight/pull/168).

---

## 3. Configuration

Everything is configured with the multi-line string in
`src/madflight_config.h` (this is madflight's runtime config format, not
`#define` macros):

```
rcl_gizmo      SERIAL
rcl_ser_bus    0
rcl_baud       115200
imu_gizmo      MPU6050
imu_i2c_adr    104
pin_imu_int    22
bar_gizmo      BMP280
bar_i2c_adr    118
mag_gizmo      HMC5883L
mag_i2c_adr    30
...
```

The pin defaults come from the board file `madflight/src/brd/drone_pico2.h`
(which is the standard Raspberry Pi Pico pinout). Values in
`madflight_config.h` are applied **after** the board file, so the config file
always wins.

Parameters can also be changed at runtime from the console and stored in flash:

```
set rcl_baud 115200
save
```

Note that `save` also reboots the flight controller, so the new values are
active after the restart.

### 3.1 Console commands (USB-CDC, 115200 baud)

| Command | Description |
|---|---|
| `help` | list all commands |
| `i2c` | scan the I2C buses - use this to check the GY-87 |
| `set <name> <value>` | change a parameter |
| `save` | store the parameters in flash and reboot |
| `diff` | show the parameters that differ from the defaults |
| `defaults` | reset the parameters to the defaults |
| `calimu` | accelerometer calibration (keep the aircraft level and still) |
| `calmag` | magnetometer calibration (rotate the aircraft in a figure of 8) |
| `calradio` | RC radio calibration (not needed for the UART control link) |
| `ps` | show the FreeRTOS task list and its CPU usage |
| `res` | show the memory and peripheral usage |
| `reboot` | restart the flight controller |

### 3.2 Live print commands

These print a value every 100 ms until the next command is typed:

| Command | Description |
|---|---|
| `po` | overview of all modules |
| `ppwm` | raw radio PWM values per channel |
| `prcl` | scaled radio sticks (-1 .. 1) |
| `pimu` | IMU loop timing (the `miss%` column should stay below 1%) |
| `pgyr` / `pacc` / `pmag` | filtered gyro, accelerometer and magnetometer |
| `pahr` / `pah` | roll, pitch and yaw in degrees |
| `ppid` / `ppidr` / `ppidp` / `ppidy` / `ppidt` | PID outputs per axis |
| `pout` | motor and servo outputs (0 .. 1) |
| `pbat` / `pbar` / `palt` | battery, barometer and altitude estimator |
| `pall` | enable all of the above |

`pmag` and `pahr` are the most useful when setting up the magnetometer, see
section 7.

The console also prints the startup banner, the I2C scan and the module
summary, so check it after every flash.

---

## 4. The drone link protocol

Two independent UART links are used. Both use the same frame layout, so a
single host tool can speak both.

```
  offset  size  field
  0       1     0xA5        sync byte 1
  1       1     0x5A        sync byte 2
  2       1     message id
  3       2     payload length in bytes (little endian)
  5       N     payload
  5+N     2     CRC16-CCITT (init 0xFFFF, poly 0x1021) over bytes [2 .. 4+N]
```

The CRC is CRC16-CCITT-FALSE, the same function as madflight's `tbx_crc16()`.
All multi-byte fields are little endian.

### 4.1 Control link (host &rarr; flight controller, UART0)

Implemented by the madflight `SERIAL` radio link gizmo, see
`madflight/src/rcl/RclGizmoSerial.h`.

| Id | Command | Payload |
|---|---|---|
| `0x01` | `SET_CHANNELS` | `N` x `uint16` PWM in microseconds, channel 1 first, `N` &le; 20 |
| `0x02` | `SET_STICKS` | `float32` roll, `float32` pitch, `float32` yaw, `float32` throttle, `uint8` arm, `uint8` flight mode |
| `0x03` | `PING` | none, keepalive |

Stick ranges:

| Stick | Range | Direction |
|---|---|---|
| roll | -1.0 .. +1.0 | -1.0 = left, +1.0 = right |
| pitch | -1.0 .. +1.0 | -1.0 = stick forward (nose down), +1.0 = stick back (nose up) |
| yaw | -1.0 .. +1.0 | -1.0 = left, +1.0 = right |
| throttle | 0.0 .. 1.0 | 0.0 = idle, 1.0 = full |

**Channel mapping** (the default AETR layout, used by `SET_CHANNELS` and by
the stick mode after conversion):

| Channel | Function | PWM range |
|---|---|---|
| 1 | roll | 1100 (left) .. 1900 (right) |
| 2 | pitch | 1100 (pulled) .. 1900 (pushed) |
| 3 | throttle | 1100 (idle) .. 1900 (full) |
| 4 | yaw | 1100 (left) .. 1900 (right) |
| 5 | arm switch | 1600..2500 = armed, &lt; 1600 = disarmed |
| 6 | flight mode | 1165 + 130 x index (0..5) |

**Link timeout:** the flight controller declares the control link lost when no
frame has arrived for 3 seconds, and disarms. Send at least one frame every
3 seconds; 10 Hz is typical, 20 Hz is used by the host tool.

**Arming:** the flight controller only arms when the throttle is at zero and
the arm switch goes from disarmed to armed. In stick mode the host tool
emulates that switch, so `--arm` requires a zero throttle.

### 4.2 Telemetry link (flight controller &rarr; host, UART1)

Implemented by `src/telemetry.cpp`.

| Id | Message | Rate | Payload size |
|---|---|---|---|
| `0x01` | `STATUS` | 10 Hz | 28 bytes |
| `0x02` | `IMU` | 50 Hz | 28 bytes |
| `0x03` | `ATTITUDE` | 25 Hz | 28 bytes |
| `0x04` | `ALTITUDE` | 10 Hz | 24 bytes |
| `0x05` | `MAG` | 10 Hz | 12 bytes |
| `0x06` | `RC` | 10 Hz | 40 bytes |
| `0x07` | `BATTERY` | 2 Hz | 20 bytes |
| `0x08` | `MOTORS` | 10 Hz | 36 bytes |
| `0x09` | `PID` | 10 Hz | 48 bytes |
| `0x0A` | `EVENT` | on demand | 3 + N bytes |

Total link load is about 5 kB/s, roughly 45% of the 115200 baud link.

#### `STATUS` (0x01)

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | uptime [ms] |
| 4 | 1 | status flags |
| 5 | 1 | flight mode index |
| 6 | 2 | IMU sample rate [Hz] |
| 8 | 4 | number of IMU updates |
| 12 | 4 | number of IMU overruns |
| 16 | 4 | number of control link updates |
| 20 | 4 | telemetry frames sent |
| 24 | 4 | telemetry frames dropped |

Status flags: `0x01` armed, `0x02` control link connected, `0x04` IMU detected,
`0x08` barometer detected, `0x10` magnetometer detected, `0x20` battery monitor
detected, `0x40` control link error.

#### `IMU` (0x02)

| Offset | Size | Field |
|---|---|---|
| 0, 4, 8 | 4 each | acceleration north, east, down [G] |
| 12, 16, 20 | 4 each | rotation north, east, down [deg/s] |
| 24 | 4 | temperature [C] |

#### `ATTITUDE` (0x03)

| Offset | Size | Field |
|---|---|---|
| 0, 4, 8 | 4 each | roll, pitch, yaw [deg] |
| 12, 16, 20, 24 | 4 each | quaternion w, x, y, z |

#### `ALTITUDE` (0x04)

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | barometric altitude [m] |
| 4 | 4 | ground level [m] |
| 8 | 4 | altitude estimate [m] |
| 12 | 4 | vertical speed [m/s] |
| 16 | 4 | pressure [Pa] |
| 20 | 4 | temperature [C] |

#### `MAG` (0x05)

| Offset | Size | Field |
|---|---|---|
| 0, 4, 8 | 4 each | magnetic flux north, east, down [uT] |

#### `RC` (0x06)

| Offset | Size | Field |
|---|---|---|
| 0, 4, 8, 12, 16 | 4 each | roll, pitch, yaw, throttle, vspeed sticks |
| 20 | 1 | armed |
| 21 | 1 | flight mode index |
| 22 | 1 | reserved |
| 23 | 1 | reserved |
| 24..39 | 2 each | channel 1..8 PWM [us] |

#### `BATTERY` (0x07)

| Offset | Size | Field |
|---|---|---|
| 0, 4, 8, 12, 16 | 4 each | voltage [V], current [A], power [W], used [mAh], used [Wh] |

#### `MOTORS` (0x08)

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | armed |
| 1 | 1 | output mode (0 = disarmed, 1 = armed, 2 = test motor) |
| 2 | 1 | number of motors |
| 3 | 1 | reserved |
| 4..35 | 4 each | motor 1..8 command [0..1] |

#### `PID` (0x09)

12 `float32` values, 3 per axis (roll, pitch, yaw, throttle):
setpoint, actual and the summed controller output.

#### `EVENT` (0x0A)

`uint8` event id, `uint8` text length, then the ASCII text.
Event ids: `0x01` text, `0x02` armed, `0x03` disarmed, `0x04` error.

---

## 5. Host tool

`tools/gcs.py` is a ground station script that decodes the telemetry and sends
control frames. It needs `pyserial` (`pip install pyserial`).

### 5.1 Watch the telemetry

```bash
python3 tools/gcs.py --telemetry /dev/ttyUSB0
```

```
STATUS   up=12.3s flags=ARMED|RCL_LINK|IMU_OK|BAR_OK|MAG_OK mode=0 imu_rate=1000 imu_upd=12300 over=0 rcl_upd=250 sent=412 drop=0
IMU      a=(+0.001,-0.002,+0.998)G g=(+0.1,-0.1,+0.0)d/s t=28.4C
ATTITUDE roll=+0.12 pitch=-0.34 yaw=+45.00
ALTITUDE alt=+0.02m v=+0.00m/s bar=+0.05m press=101325.0Pa t=28.4C
MAG      n=+21.4 e=-2.1 d=+38.9 uT
RC       roll=+0.000 pitch=+0.000 yaw=+0.000 thr=+0.000 vs=+0.000 armed=1 mode=0 pwm=[1500 1500 1160 1500 1600 1165]
MOTORS   armed=1 mode=1  1:0.250 2:0.250 3:0.250 4:0.250
```

Note that in stick mode the arm channel shows 1600 (`rcl_arm_min`) when the
drone is armed, and the throttle channel shows 1160 (`rcl_thr_pull` + the 60 us
deadband) at idle.

### 5.2 Send control frames

```bash
# normalized sticks at 20 Hz for 10 seconds
python3 tools/gcs.py --control /dev/ttyUSB1 --sticks 0 0 0 0 --duration 10

# arm (throttle must be zero)
python3 tools/gcs.py --control /dev/ttyUSB1 --sticks 0 0 0 0 --arm --duration 5

# raw channel PWM values
python3 tools/gcs.py --control /dev/ttyUSB1 --channels 1500 1500 1160 1500 1000 1165

# keep the link alive only
python3 tools/gcs.py --control /dev/ttyUSB1 --ping
```

`--sticks` and `--channels` are mutually exclusive. `--flightmode N` selects
the flight mode (0..5), `--rate` sets the frame rate and `--duration` limits
how long the tool runs (0 = until Ctrl-C).

The tool refuses to arm with a non-zero throttle.

### 5.3 Protocol self test

```bash
python3 tools/test_protocol.py
```

The test verifies the CRC implementation against the values produced by
madflight's `tbx_crc16()`, checks the frame layout, round-trips every control
frame through the parser and decodes a synthetic payload for every telemetry
message.

---

## 6. First-time setup

1. **Flash and check the console.** The startup banner lists the detected
   modules. All three sensors must be detected:

   ```
   IMU: MPU6050 i2c_adr:0x68 ...
   BAR: BMP280 i2c_adr=0x76 SensorID=0x58
   MAG: HMC5883L i2c_adr=0x1E detect=1 chip_id=0x48
   ```

   Run `i2c` if anything is missing. A missing MPU6050 makes madflight panic
   (the IMU is mandatory).

2. **Calibrate the accelerometer.** Hold the aircraft level and still, then
   run `calimu` on the console.

3. **Calibrate the magnetometer.** Run `calmag` and rotate the aircraft in a
   figure of 8 motion for about 15 seconds, in a place without metal objects
   and away from high power cables. Then `save`.

4. **Verify the heading.** This is the step that most often needs a fix, see
   section 7.

5. **Check the motor direction.** Remove the props. Arm with the throttle at
   zero and give a small amount of throttle. The motors must spin in the
   correct direction for the mixer in `src/main.cpp` to be correct.

6. **Set up the control link** and verify the stick directions with the `prcl`
   console command (roll, pitch, yaw and throttle, scaled to -1 .. 1). The same
   values are sent in the `RC` telemetry message.

---

## 7. Magnetometer heading - verification checklist

The HMC5883L and the MPU6050 sit at different positions on the GY-87 board and
their axes are not aligned, so the heading needs to be verified once. The
driver has two axis maps, selected with the `HMC5883L_AXIS_MAP` macro in
`madflight/src/mag/HMC5883L/HMC5883L.h`:

| Map | Definition |
|---|---|
| `HMC5883L_AXIS_NWU` (default) | `n = +y`, `w = -x`, `u = -z` - the physical orientation from the datasheet |
| `HMC5883L_AXIS_RAW` | `n = +x`, `w = +y`, `u = +z` - the raw chip axes |

Checklist:

1. Place the aircraft on a flat surface, motors removed, pointing **north**.
   Use a phone compass to verify the direction.
2. Watch the `MAG` telemetry message. The **north** component (`mx`) must be
   strongly positive (the Earth's field in Europe/USA is 45..55 uT) and the
   **east** component (`my`) must be close to zero.
3. If `mx` is negative or `my` is large, the axis map is wrong:
   * `mx` negative and `my` small &rarr; the map is rotated by 180&deg;:
     switch to the other map, or use `set mag_align CW180`.
   * `mx` small and `my` large &rarr; the north and east axes are swapped:
     use `set mag_align CW90` or `CW270`.
4. Rotate the aircraft 90&deg; clockwise (nose pointing east). `mx` must now be
   close to zero and `my` must be strongly **negative** (east is the negative
   y axis in madflight's NED convention).
5. Rotate it 180&deg; (nose pointing south). `mx` must be strongly negative.
6. Finally check the reported heading: the `ATTITUDE` message's `yaw` must
   increase when the aircraft is rotated **clockwise** (viewed from above).

The `mag_align` parameter (`CW0`, `CW90`, `CW180`, `CW270`, `CW0FLIP`,
`CW90FLIP`, `CW180FLIP`, `CW270FLIP`) compensates for the rotation of the
sensor board on the airframe. Use `set mag_align CW90` followed by `save`
(which stores the parameters and reboots).

---

## 8. Optional: battery monitoring

The battery monitor is disabled by default. To enable it, uncomment these
lines in `src/madflight_config.h`:

```
bat_gizmo      ADC
pin_bat_i      27     // adc: current
pin_bat_v      28     // adc: voltage
bat_cal_v      1      // adc: voltage scale, value is: actual_voltage_in_v / adc_reading
```

On the RP2350 the ADC channels are `GPIO26 = ADC0`, `GPIO27 = ADC1`,
`GPIO28 = ADC2`, `GPIO29 = ADC3`, and `ADC4` is the internal temperature
sensor (it has no GPIO). The wiring above therefore uses **ADC1** for the
current sense and **ADC2** for the voltage sense.

* **Voltage:** connect the battery positive to GPIO28 through a voltage
  divider that keeps the ADC input below 3.3 V. A 20k (top) / 10k (bottom)
  divider works for a 2S battery (8.4 V -> 2.8 V at the pin).
* **Current:** connect GPIO27 to the current sense output of an INA219/INA226
  style sensor. With a plain resistor divider this is not possible, a current
  sensor module is required.

### 8.1 How to calculate `bat_cal_v`

madflight reads the ADC with `analogReadResolution(16)`, so the raw reading
is 0..65535:

```
adc_reading = V_adc / 3.3 * 65535
```

`bat_cal_v` is the factor that converts the raw reading into volts, so

```
bat_cal_v = V_battery / adc_reading = 3.3 / (k * 65535)
```

where `k = V_adc / V_battery` is the ratio of your divider. For a 2S battery
with a 20k/10k divider `k = 1/3`, which gives `bat_cal_v = 1.51e-4`.

```
set bat_cal_v 0.000151
save
```

Verify the result with the `pbat` console command or the `BATTERY` telemetry
message, and fine tune `bat_cal_v` against a multimeter.

### 8.2 Known upstream quirk

`BatGizmoADC::update()` in madflight scales the **current** reading with
`bat_cal_v` as well:

```
state->i = config->adc_cal_v * analogRead(config->adc_pin_i);
```

It should use `adc_cal_i`, so `bat_cal_i` currently has no effect and the
current is reported with the voltage scale factor. Either fix that line in
`madflight/src/bat/BatGizmoADC.h`, or use an INA226/INA228 module instead
(`bat_gizmo INA226`, `bat_i2c_bus 0`, `bat_i2c_adr 64`), where the current
scale works as documented and `bat_cal_i` is the shunt resistance in ohm.

---

## 9. Project layout

```
Drone/
├── .gitmodules              madflight submodule
├── madflight/               the madflight library (submodule)
└── firmware/
    ├── platformio.ini       PlatformIO project, environment RP2350A
    ├── src/
    │   ├── madflight_config.h  pins, sensors and link settings
    │   ├── main.cpp            the only file that includes <madflight.h>
    │   ├── protocol.h          the drone link protocol
    │   └── telemetry.h/.cpp    the telemetry task
    └── tools/
        ├── gen_param.py        regenerates madflight's param.yaml.h
        ├── pre_gen_param.py    PlatformIO pre-build hook
        ├── gcs.py              host ground station tool
        └── test_protocol.py    protocol self test
```

`madflight.h` contains the madflight task definitions and deliberately has no
`#pragma once`, so it must be included in exactly **one** translation unit.
`src/main.cpp` is that translation unit. `src/telemetry.cpp` includes the
individual module headers (`imu/imu.h`, `ahr/ahr.h`, ...) instead, all of
which are `#pragma once` safe.

### 9.1 Changes to the madflight library

| File | Change |
|---|---|
| `src/mag/HMC5883L/HMC5883L.h` | new driver for the Honeywell HMC5883L (the QMC5883L is a different chip with a different register map) |
| `src/mag/MagGizmoHMC5883L.h` | new magnetometer gizmo |
| `src/mag/mag.cpp` | register the new gizmo |
| `src/rcl/RclGizmoSerial.h` | new radio link gizmo: binary RC protocol over a UART |
| `src/rcl/rcl.cpp` | register the new gizmo |
| `src/rcl/rcl.h` | move `RCL_THROTTLE_DEADBAND` here so the gizmo can use it |
| `src/brd/drone_pico2.h` | board file for this project |
| `src/cfg/param.yaml` | add `HMC5883L` to `mag_gizmo` and `SERIAL` to `rcl_gizmo` |
| `src/cfg/param.yaml.h` | regenerated from `param.yaml` |

Both new enum values were appended at the **end** of their option lists, so the
existing enum values keep their numbers and saved configurations stay valid.

These changes live on the branch `feature/pico2-gy87` of the madflight fork
[btechioi/madflight](https://github.com/btechioi/madflight), which is what the
`madflight/` submodule points at. They are also submitted upstream in
[PR #168](https://github.com/qqqlab/madflight/pull/168).

---

## 10. Notes and limitations

* The two links are independent UARTs, so the control link and the telemetry
  link cannot interfere with each other. The USB console is a third,
  independent channel.
* The IMU runs on core 1 at 1000 Hz, the telemetry task runs on core 0 at a
  low priority and never blocks the IMU task.
* The barometer and the magnetometer are read from madflight's `sensor_task`
  on core 0, while the IMU is read from core 1. Both use the same I2C bus, so
  there can be I2C contention between the two cores. The madflight drivers use
  short transactions, but if you see sporadic I2C errors, lower the IMU rate or
  move the magnetometer to I2C1 (GPIO10 = SDA, GPIO11 = SCL).
* The mixer is madflight's quadcopter mix (Betaflight motor order M1..M4),
  see `out_Mixer()` in `src/main.cpp`. If the props are mounted so that they
  spin inwards instead of outwards, reverse the third (yaw) column of the mix
  matrix - the alternative matrix is in the comment above it.
* The magnetometer is on a separate chip from the IMU, so the yaw estimate
  drifts slowly and needs the magnetometer to correct it. If the heading is
  wrong, go through the checklist in section 7.
* The telemetry link is one way. The flight controller does not read anything
  from UART1.
