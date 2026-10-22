/*==========================================================================================================================
 Telemetry - sends flight data over a UART to a ground station
===========================================================================================================================

The telemetry task runs on core 0 at a low priority. It never blocks the IMU
task (core 1) and it never touches the control link (UART0).

See protocol.h for the frame layout and the message descriptions.

NOTE: this header deliberately does NOT include <madflight.h>. The madflight
header contains the task definitions and must only be included in one
translation unit (src/main.cpp). Here we include the individual module
headers, which are all "#pragma once" safe.

==========================================================================================================================*/

#pragma once

#include <stdint.h>
#include <Arduino.h>
#include <FreeRTOS.h>
#include <queue.h>

#include "hal/MF_Serial.h"
#include "protocol.h"

class Telemetry {
  public:
    //setup the telemetry link, ser must be a valid MF_Serial (e.g. hal_get_ser_bus(1, 115200))
    void begin(MF_Serial *ser);

    //start the telemetry task (call after madflight_setup())
    void start();

    //queue a text event, safe to call from any task
    void event(uint8_t id, const char *text);

    //number of frames sent / dropped
    uint32_t frame_count() { return _frames_sent; }
    uint32_t drop_count() { return _frames_dropped; }

  private:
    static void task_entry(void *pvParameters);
    void run();

    void send(uint8_t id, const uint8_t *payload, uint16_t len);
    void send_message(uint8_t id);

    void send_status();
    void send_imu();
    void send_attitude();
    void send_altitude();
    void send_mag();
    void send_rc();
    void send_battery();
    void send_motors();
    void send_pid();

    MF_Serial *_ser = nullptr;

    uint8_t _txbuf[TEL_MAX_PAYLOAD + 8];
    uint8_t _payload[TEL_MAX_PAYLOAD];

    //pending event messages
    QueueHandle_t _event_queue = nullptr;

    //transmit statistics
    volatile uint32_t _frames_sent = 0;
    volatile uint32_t _frames_dropped = 0;

    //schedule
    struct schedule_t {
      uint8_t id;
      uint32_t interval_ms;
      uint32_t next_ms;
    };
    schedule_t _schedule[9];
    uint8_t _schedule_cnt = 0;
};

//the global telemetry instance
extern Telemetry telemetry;
