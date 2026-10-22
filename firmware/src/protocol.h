/*==========================================================================================================================
 Drone link protocol
===========================================================================================================================

Two independent links are used:

  * CONTROL LINK  - UART0 (GPIO1 = RX).  Flight controller receives, host transmits.
                    Implemented by the madflight "SERIAL" rcl gizmo, see
                    madflight/src/rcl/RclGizmoSerial.h

  * TELEMETRY LINK - UART1 (GPIO8 = TX). Flight controller transmits, host receives.
                    Implemented by src/telemetry.cpp, described below.

Both links use the same frame layout, so a single host tool can speak both.

Frame layout (all multi byte fields little endian)
--------------------------------------------------

    offset  size  field
    0       1     0xA5          sync byte 1
    1       1     0x5A          sync byte 2
    2       1     message id
    3       2     payload length in bytes
    5       N     payload
    5+N     2     CRC16-CCITT (init 0xFFFF, poly 0x1021) over bytes [2 .. 4+N]

The CRC is CRC16-CCITT-FALSE, the same function as madflight's tbx_crc16().

Telemetry messages (flight controller -> host)
----------------------------------------------

  0x01 STATUS     28 bytes  see send_status()
  0x02 IMU        28 bytes  see send_imu()
  0x03 ATTITUDE   28 bytes  see send_attitude()
  0x04 ALTITUDE   24 bytes  see send_altitude()
  0x05 MAG        12 bytes  see send_mag()
  0x06 RC         40 bytes  see send_rc()
  0x07 BATTERY    20 bytes  see send_battery()
  0x08 MOTORS     36 bytes  see send_motors()
  0x09 PID        48 bytes  see send_pid()
  0x0A EVENT      3+N bytes event id, length, ascii text

Control messages (host -> flight controller)
--------------------------------------------

  0x01 SET_CHANNELS  N * uint16 PWM in [us], channel 1 first, N <= 20
  0x02 SET_STICKS    4 * float32 (roll, pitch, yaw, throttle)
                      + uint8 arm + uint8 flight mode index
  0x03 PING           no payload, keepalive

===========================================================================================================================*/

#pragma once

#include <stdint.h>

//-------------------------------------------------------------------------------------------------
// Framing
//-------------------------------------------------------------------------------------------------
#define TEL_SYNC1            0xA5
#define TEL_SYNC2            0x5A
#define TEL_MAX_PAYLOAD      160 //maximum payload of a telemetry frame

//-------------------------------------------------------------------------------------------------
// Expected payload sizes - checked at runtime by telemetry.cpp
//-------------------------------------------------------------------------------------------------
#define TEL_SIZE_STATUS      28
#define TEL_SIZE_IMU         28
#define TEL_SIZE_ATTITUDE    28
#define TEL_SIZE_ALTITUDE    24
#define TEL_SIZE_MAG         12
#define TEL_SIZE_RC          40
#define TEL_SIZE_BATTERY     20
#define TEL_SIZE_MOTORS      36
#define TEL_SIZE_PID         48
#define TEL_SIZE_EVENT       0 //variable size: event id + length + text

//-------------------------------------------------------------------------------------------------
// Telemetry message ids (flight controller -> host)
//-------------------------------------------------------------------------------------------------
#define TEL_MSG_STATUS       0x01
#define TEL_MSG_IMU          0x02
#define TEL_MSG_ATTITUDE     0x03
#define TEL_MSG_ALTITUDE     0x04
#define TEL_MSG_MAG          0x05
#define TEL_MSG_RC           0x06
#define TEL_MSG_BATTERY      0x07
#define TEL_MSG_MOTORS       0x08
#define TEL_MSG_PID          0x09
#define TEL_MSG_EVENT        0x0A

//-------------------------------------------------------------------------------------------------
// Status flags
//-------------------------------------------------------------------------------------------------
#define TEL_FLAG_ARMED       0x01 // flight controller is armed
#define TEL_FLAG_RCL_LINK    0x02 // control link is connected
#define TEL_FLAG_IMU_OK      0x04 // IMU detected
#define TEL_FLAG_BAR_OK      0x08 // barometer detected
#define TEL_FLAG_MAG_OK      0x10 // magnetometer detected
#define TEL_FLAG_BAT_OK      0x20 // battery monitor detected
#define TEL_FLAG_RCL_FAULT   0x40 // control link error (crc / framing)

//-------------------------------------------------------------------------------------------------
// Event ids
//-------------------------------------------------------------------------------------------------
#define TEL_EVENT_TEXT       0x01 // generic text message
#define TEL_EVENT_ARMED      0x02 // "ARMED"
#define TEL_EVENT_DISARMED   0x03 // "DISARMED"
#define TEL_EVENT_ERROR      0x04 // "ERROR: ..."

//-------------------------------------------------------------------------------------------------
// Control message ids (host -> flight controller)
//-------------------------------------------------------------------------------------------------
#define TEL_CMD_SET_CHANNELS 0x01
#define TEL_CMD_SET_STICKS   0x02
#define TEL_CMD_PING         0x03
