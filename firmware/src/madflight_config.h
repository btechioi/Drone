/*==========================================================================================================================
                                              MADFLIGHT CONFIG
===========================================================================================================================

Flight controller firmware for a Raspberry Pi Pico 2 (RP2350) with a GY-87 10DOF
sensor board (MPU6050 + BMP280 + HMC5883L).

Wiring (Raspberry Pi Pico 2 GPIO numbers)
-----------------------------------------
    GY-87 I2C      : SDA = GPIO20, SCL = GPIO21
    MPU6050 DRDY   : GPIO22   (required! madflight panics when the IMU interrupt never fires)
    RC input UART0 : RX = GPIO1 (flight controller <- host), TX = GPIO0 (flight controller -> host)
    Telemetry UART1: TX = GPIO8 (flight controller -> host), RX = GPIO9 (host -> flight controller, unused)
    Motor 1..4     : GPIO2, GPIO3, GPIO4, GPIO5   (PWM slices 1 and 2)
    Status LED     : GPIO25
    USB-CDC console: the native USB port (madflight CLI, 115200 baud)

The configuration is loaded from two strings: first from madflight_board in the
board header in /brd/, and then from madflight_config in this file. If the same
setting occurs more than once, the last one is applied.

The strings are multi-line raw strings with a key-value list. Anything after
'#' or '/' is ignored as a comment.

Pins and spi/i2c/serial busses use zero-based numbering, i.e. "gps_ser_bus 0"
connects the GPS to the first serial bus with pins pin_ser0_tx and pin_ser0_rx.
Pins use GPIO numbers, not physical pin numbers. Use -1 to disable a pin or bus.

You can also modify the configuration from the CLI, for example
"set imu_gizmo MPU6050", then use "save" to save the config to eeprom and
reboot to use the new config.

If things do not work as expected, have a good look at the startup messages!

===========================================================================================================================*/

// define the board header to use, or comment out for none
#define MF_BOARD "brd/drone_pico2.h"

const char madflight_config[] = R""(

//--- RCL --- Remote Control Link
// Control frames are received on serial bus 0 (UART0, GPIO1 = RX).
// Frame format: A5 5A | cmd u8 | len u16 | payload | crc16  (see madflight/src/rcl/RclGizmoSerial.h)
rcl_gizmo      SERIAL  // options: NONE, MAVLINK, CRSF, SBUS, SBUS_NOT_INV, DSM, PPM, IBUS, SERIAL
rcl_ser_bus    0
rcl_baud       115200
rcl_num_ch     16
rcl_deadband   0
pin_rcl_ppm   -1

//flightmode mapping from 2/3/6-pos switch to flight mode
rcl_flt0 RATE
rcl_flt1 RATE
rcl_flt2 RATE
rcl_flt3 RATE
rcl_flt4 RATE
rcl_flt5 ANGLE

//--- IMU --- MPU6050 on I2C bus 0 (0x68 = decimal 104)
imu_gizmo      MPU6050  // options: NONE, BMI270, MPU6000, MPU6050, MPU6500, MPU9150, MPU9250, ICM45686, ...
imu_bus_type   I2C     // options: SPI, I2C
imu_i2c_bus    0
imu_i2c_adr    104     // 0x68 MPU6050 AD0 low (default on the GY-87). Decimal number only!
imu_rate       1000    // [Hz]
imu_align      CW0     // options: CW0, CW90, CW180, CW270, CW0FLIP, CW90FLIP, CW180FLIP, CW270FLIP
pin_imu_int    22      // MPU6050 data ready interrupt - REQUIRED

//--- BAR --- Barometer BMP280 on I2C bus 0 (0x76 = decimal 118)
bar_gizmo      BMP280   // options: NONE, BMP280, BMP388, BMP390, MS5611, HP203B, BMP580, DPS310
bar_i2c_bus    0
bar_i2c_adr    118     // 0x76 BMP280 SDO low (default on the GY-87). Decimal number only!

//--- MAG --- Magnetometer HMC5883L on I2C bus 0 (0x1E = decimal 30)
mag_gizmo      HMC5883L // options: NONE, QMC5883, QMC6309, RM3100, QMC5883L, QMC5883P, MMC5603, BMM150, HMC5883L
mag_i2c_bus    0
mag_i2c_adr    30      // 0x1E HMC5883L ADDR low (default on the GY-87). Decimal number only!
mag_align      CW0     // options: CW0, CW90, CW180, CW270, CW0FLIP, CW90FLIP, CW180FLIP, CW270FLIP

//--- BAT --- Battery Monitor (disabled by default, see the README for the ADC wiring)
//bat_gizmo      ADC     // options: NONE, ADC, INA226, INA228
//pin_bat_i      27     // adc: current
//pin_bat_v      28     // adc: voltage
//bat_cal_v      1      // adc: voltage scale, value is: actual_voltage_in_v / adc_reading

//--- GPS --- not used
gps_gizmo      NONE
gps_ser_bus   -1

//--- BBX --- Blackbox datalogger (not used)
bbx_gizmo      NONE

//--- RDR --- Radar (not used)
rdr_gizmo      NONE
rdr_ser_bus   -1

//--- OFL --- Optical flow (not used)
ofl_gizmo      NONE
ofl_ser_bus   -1

//--- LED ---
led_gizmo     HIGH_IS_ON // options: NONE, HIGH_IS_ON, LOW_IS_ON, RGB
pin_led       25

//--- AHR --- Attitude estimation
ahr_gizmo      MAHONY   // options: MAHONY, MAHONY_BF, MADGWICK, VQF

//--- Serial bus 0 (RC control input) ---
pin_ser0_rx    1
pin_ser0_tx    0
pin_ser0_inv  -1

//--- Serial bus 1 (telemetry output) ---
pin_ser1_rx    9
pin_ser1_tx    8
pin_ser1_inv  -1

//--- I2C bus 0 (GY-87) ---
pin_i2c0_sda  20
pin_i2c0_scl  21

//--- OUT Pins (motor outputs 1..4) ---
pin_out0       2
pin_out1       3
pin_out2       4
pin_out3       5
pin_out4      -1
pin_out5      -1
pin_out6      -1
pin_out7      -1
pin_out8      -1
pin_out9      -1
pin_out10     -1
pin_out11     -1
pin_out12     -1
pin_out13     -1
pin_out14     -1
pin_out15     -1

)""; // End of madflight_config


//========================================================================================================================//
//                                               COMPILER OPTIONS                                                         //
//========================================================================================================================//

// Reset config eeprom to defaults (uncomment this, upload, execute, then comment out, and upload again)
//#define MF_CONFIG_CLEAR

// Uncomment to reduce startup delay
//#define MF_DEBUG
