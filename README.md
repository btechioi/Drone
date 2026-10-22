# Drone

Drone firmware for the **Raspberry Pi Pico 2 (RP2350)** with a **GY-87 10DOF**
sensor board (MPU6050 IMU + BMP280 barometer + HMC5883L magnetometer), built on
the [madflight](https://madflight.com) flight controller library.

```
Drone/
├── madflight/          git submodule -> btechioi/madflight (branch feature/pico2-gy87)
└── firmware/           the flight controller project
```

## Features

| | |
|---|---|
| MCU | Raspberry Pi Pico 2 (RP2350A) |
| Sensors | GY-87 10DOF: MPU6050 + BMP280 + HMC5883L, all on I2C0 |
| Control link | binary RC protocol over UART0 (GPIO0/1), 115200 baud |
| Telemetry link | binary flight data over UART1 (GPIO8/9), 115200 baud |
| Console | madflight CLI over USB-CDC |
| Motors | 4 outputs, GPIO2..GPIO5, 400 Hz PWM, 950-2000 us |

## Quick start

```bash
git clone --recurse-submodules https://github.com/btechioi/Drone.git
cd Drone/firmware
pio run -e RP2350A                # build
pio run -e RP2350A -t upload      # flash over USB
pio device monitor -e RP2350A    # console (type 'help')
```

Host ground station tool:

```bash
python3 tools/gcs.py --telemetry /dev/ttyUSB0        # watch telemetry
python3 tools/gcs.py --control /dev/ttyUSB1 --sticks 0 0 0 0 --arm --duration 5
python3 tools/test_protocol.py                       # protocol self test
```

**Full documentation, wiring tables and the protocol reference are in
[firmware/README.md](firmware/README.md).**

## Library changes

The HMC5883L driver, the `SERIAL` radio link gizmo and the Pico 2 board file
are added to madflight on the branch `feature/pico2-gy87` of the fork
[btechioi/madflight](https://github.com/btechioi/madflight), which is what the
`madflight/` submodule points at.
