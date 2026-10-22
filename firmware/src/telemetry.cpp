/*==========================================================================================================================
 Telemetry - sends flight data over a UART to a ground station
===========================================================================================================================

See protocol.h for the frame layout and the message descriptions.

===========================================================================================================================*/

#include "telemetry.h"

//madflight modules - individual headers only, <madflight.h> may only be
//included by src/main.cpp because it defines the madflight tasks
#include "imu/imu.h"
#include "ahr/ahr.h"
#include "alt/alt.h"
#include "bar/bar.h"
#include "mag/mag.h"
#include "bat/bat.h"
#include "rcl/rcl.h"
#include "out/out.h"
#include "pid/pid.h"
#include "hal/hal.h"
#include "tbx/tbx_crc.h"

//the global telemetry instance
Telemetry telemetry;

//-------------------------------------------------------------------------------------------------
// Little endian byte writer
//-------------------------------------------------------------------------------------------------
namespace {

class ByteWriter {
  public:
    ByteWriter(uint8_t *buf, uint16_t cap) : _buf(buf), _cap(cap) {}

    void u8(uint8_t v) {
      if(_pos + 1 > _cap) { _overflow = true; return; }
      _buf[_pos++] = v;
    }

    void u16(uint16_t v) {
      if(_pos + 2 > _cap) { _overflow = true; return; }
      _buf[_pos++] = (uint8_t)(v & 0xFF);
      _buf[_pos++] = (uint8_t)((v >> 8) & 0xFF);
    }

    void u32(uint32_t v) {
      if(_pos + 4 > _cap) { _overflow = true; return; }
      _buf[_pos++] = (uint8_t)(v & 0xFF);
      _buf[_pos++] = (uint8_t)((v >> 8) & 0xFF);
      _buf[_pos++] = (uint8_t)((v >> 16) & 0xFF);
      _buf[_pos++] = (uint8_t)((v >> 24) & 0xFF);
    }

    void f32(float v) {
      uint32_t u = 0;
      memcpy(&u, &v, sizeof(u));
      u32(u);
    }

    uint16_t size() const { return _pos; }
    bool overflow() const { return _overflow; }

  private:
    uint8_t *_buf;
    uint16_t _cap;
    uint16_t _pos = 0;
    bool _overflow = false;
};

struct event_t {
  uint8_t id;
  char text[48];
};

//the expected payload size of a telemetry message, 0 means "variable"
uint16_t expected_payload_size(uint8_t id) {
  switch(id) {
    case TEL_MSG_STATUS:    return TEL_SIZE_STATUS;
    case TEL_MSG_IMU:       return TEL_SIZE_IMU;
    case TEL_MSG_ATTITUDE:  return TEL_SIZE_ATTITUDE;
    case TEL_MSG_ALTITUDE:  return TEL_SIZE_ALTITUDE;
    case TEL_MSG_MAG:       return TEL_SIZE_MAG;
    case TEL_MSG_RC:        return TEL_SIZE_RC;
    case TEL_MSG_BATTERY:   return TEL_SIZE_BATTERY;
    case TEL_MSG_MOTORS:    return TEL_SIZE_MOTORS;
    case TEL_MSG_PID:       return TEL_SIZE_PID;
    case TEL_MSG_EVENT:     return TEL_SIZE_EVENT;
    default:                return 0;
  }
}

} //namespace

//-------------------------------------------------------------------------------------------------
// Setup
//-------------------------------------------------------------------------------------------------

void Telemetry::begin(MF_Serial *ser) {
  _ser = ser;

  //event queue
  if(!_event_queue) {
    _event_queue = xQueueCreate(8, sizeof(event_t));
  }

  //message schedule, the intervals are in milliseconds
  _schedule[_schedule_cnt++] = {TEL_MSG_IMU,       20,  0}; // 50 Hz
  _schedule[_schedule_cnt++] = {TEL_MSG_ATTITUDE,  40,  0}; // 25 Hz
  _schedule[_schedule_cnt++] = {TEL_MSG_STATUS,   100,  0}; // 10 Hz
  _schedule[_schedule_cnt++] = {TEL_MSG_ALTITUDE, 100,  0}; // 10 Hz
  _schedule[_schedule_cnt++] = {TEL_MSG_MAG,      100,  0}; // 10 Hz
  _schedule[_schedule_cnt++] = {TEL_MSG_RC,       100,  0}; // 10 Hz
  _schedule[_schedule_cnt++] = {TEL_MSG_MOTORS,   100,  0}; // 10 Hz
  _schedule[_schedule_cnt++] = {TEL_MSG_PID,      100,  0}; // 10 Hz
  _schedule[_schedule_cnt++] = {TEL_MSG_BATTERY,  500,  0}; //  2 Hz
}

void Telemetry::start() {
  //same priority as the other madflight tasks, running on core 0
  UBaseType_t prio = uxTaskPriorityGet(NULL);
  hal_xTaskCreate(task_entry, "mf_TLM", 1024, this, prio, NULL, 0);
}

void Telemetry::task_entry(void *pvParameters) {
  ((Telemetry *)pvParameters)->run();
}

//-------------------------------------------------------------------------------------------------
// Main loop
//-------------------------------------------------------------------------------------------------

void Telemetry::run() {
  for(;;) {
    uint32_t now = millis();

    //send the scheduled messages
    for(uint8_t i = 0; i < _schedule_cnt; i++) {
      if((int32_t)(now - _schedule[i].next_ms) >= 0) {
        _schedule[i].next_ms = now + _schedule[i].interval_ms;
        send_message(_schedule[i].id);
      }
    }

    //send the pending events
    event_t ev;
    while(_event_queue && xQueueReceive(_event_queue, &ev, 0) == pdTRUE) {
      uint16_t len = (uint16_t)strlen(ev.text);
      if(len > 40) len = 40;
      ByteWriter w(_payload, TEL_MAX_PAYLOAD);
      w.u8(ev.id);
      w.u8((uint8_t)len);
      for(uint16_t i = 0; i < len; i++) w.u8((uint8_t)ev.text[i]);
      send(TEL_MSG_EVENT, _payload, w.size());
    }

    vTaskDelay(1); //1ms tick, yields to the other tasks
  }
}

void Telemetry::event(uint8_t id, const char *text) {
  if(!_event_queue || !text) return;
  event_t ev;
  ev.id = id;
  strncpy(ev.text, text, sizeof(ev.text) - 1);
  ev.text[sizeof(ev.text) - 1] = '\0';
  xQueueSend(_event_queue, &ev, 0); //drop the event when the queue is full
}

//-------------------------------------------------------------------------------------------------
// Framing
//-------------------------------------------------------------------------------------------------

void Telemetry::send(uint8_t id, const uint8_t *payload, uint16_t len) {
  if(!_ser) return;
  if(len > TEL_MAX_PAYLOAD) return;

  //verify the payload size against the protocol definition, so that a change
  //in the message layout can never silently produce garbage on the wire
  uint16_t expected = expected_payload_size(id);
  if(expected && len != expected) {
    _frames_dropped++;
    return;
  }

  uint16_t n = 0;
  _txbuf[n++] = TEL_SYNC1;
  _txbuf[n++] = TEL_SYNC2;
  _txbuf[n++] = id;
  _txbuf[n++] = (uint8_t)(len & 0xFF);
  _txbuf[n++] = (uint8_t)((len >> 8) & 0xFF);
  if(len > 0) {
    memcpy(&_txbuf[n], payload, len);
    n += len;
  }

  //CRC over the message id, the length and the payload
  uint16_t crc = tbx_crc16(&_txbuf[2], 3 + len);
  _txbuf[n++] = (uint8_t)(crc & 0xFF);
  _txbuf[n++] = (uint8_t)((crc >> 8) & 0xFF);

  //wait until the transmit buffer has enough room for the whole frame
  uint32_t ts = millis();
  while(_ser->availableForWrite() < (int)n) {
    if((millis() - ts) > 50) {
      _frames_dropped++;
      return; //give up, the link is too slow
    }
    vTaskDelay(1);
  }

  if(_ser->write(_txbuf, n) == n) {
    _frames_sent++;
  }else{
    _frames_dropped++;
  }
}

void Telemetry::send_message(uint8_t id) {
  switch(id) {
    case TEL_MSG_STATUS:    send_status();    break;
    case TEL_MSG_IMU:       send_imu();       break;
    case TEL_MSG_ATTITUDE:  send_attitude();  break;
    case TEL_MSG_ALTITUDE:  send_altitude();  break;
    case TEL_MSG_MAG:       send_mag();       break;
    case TEL_MSG_RC:        send_rc();        break;
    case TEL_MSG_BATTERY:   send_battery();   break;
    case TEL_MSG_MOTORS:    send_motors();    break;
    case TEL_MSG_PID:       send_pid();       break;
    default: break;
  }
}

//-------------------------------------------------------------------------------------------------
// Messages
//-------------------------------------------------------------------------------------------------

void Telemetry::send_status() {
  uint8_t flags = 0;
  if(out.armed()) flags |= TEL_FLAG_ARMED;
  if(rcl.connected()) flags |= TEL_FLAG_RCL_LINK;
  if(imu.installed()) flags |= TEL_FLAG_IMU_OK;
  if(bar.installed()) flags |= TEL_FLAG_BAR_OK;
  if(mag.installed()) flags |= TEL_FLAG_MAG_OK;
  if(bat.installed()) flags |= TEL_FLAG_BAT_OK;

  ByteWriter w(_payload, TEL_MAX_PAYLOAD);
  w.u32((uint32_t)millis());          //0  uptime [ms]
  w.u8(flags);                        //4  status flags
  w.u8(rcl.flightmode_idx);           //5  flight mode index
  w.u16((uint16_t)imu.getSampleRate()); //6 IMU sample rate [Hz]
  w.u32(imu.update_cnt);              //8  number of IMU updates
  w.u32(imu.overrun_cnt);             //12 number of IMU overruns
  w.u32((uint32_t)rcl.update_count());//16 number of control link updates
  w.u32(_frames_sent);                //20 number of telemetry frames sent
  w.u32(_frames_dropped);             //24 number of telemetry frames dropped
  send(TEL_MSG_STATUS, _payload, w.size());
}

void Telemetry::send_imu() {
  ByteWriter w(_payload, TEL_MAX_PAYLOAD);
  w.f32(imu.ax);                       //0 acceleration north [G]
  w.f32(imu.ay);                       //4 acceleration east [G]
  w.f32(imu.az);                       //8 acceleration down [G]
  w.f32(imu.gx);                       //12 rotation north [deg/s]
  w.f32(imu.gy);                       //16 rotation east [deg/s]
  w.f32(imu.gz);                       //20 rotation down [deg/s]
  w.f32(imu.temp);                     //24 temperature [C]
  send(TEL_MSG_IMU, _payload, w.size());
}

void Telemetry::send_attitude() {
  ByteWriter w(_payload, TEL_MAX_PAYLOAD);
  w.f32(ahr.roll);                     //0 roll [deg]
  w.f32(ahr.pitch);                    //4 pitch [deg]
  w.f32(ahr.yaw);                      //8 yaw [deg]
  w.f32(ahr.q[0]);                     //12 quaternion w
  w.f32(ahr.q[1]);                     //16 quaternion x
  w.f32(ahr.q[2]);                     //20 quaternion y
  w.f32(ahr.q[3]);                     //24 quaternion z
  send(TEL_MSG_ATTITUDE, _payload, w.size());
}

void Telemetry::send_altitude() {
  ByteWriter w(_payload, TEL_MAX_PAYLOAD);
  w.f32(bar.alt);                      //0 barometric altitude [m]
  w.f32(bar.ground_level);             //4 ground level [m]
  w.f32(alt.getH());                   //8 altitude estimate [m]
  w.f32(alt.getV());                   //12 vertical speed [m/s]
  w.f32(bar.press);                    //16 pressure [Pa]
  w.f32(bar.temp);                     //20 temperature [C]
  send(TEL_MSG_ALTITUDE, _payload, w.size());
}

void Telemetry::send_mag() {
  ByteWriter w(_payload, TEL_MAX_PAYLOAD);
  w.f32(mag.mx);                       //0 magnetic flux north [uT]
  w.f32(mag.my);                       //4 magnetic flux east [uT]
  w.f32(mag.mz);                       //8 magnetic flux down [uT]
  send(TEL_MSG_MAG, _payload, w.size());
}

void Telemetry::send_rc() {
  ByteWriter w(_payload, TEL_MAX_PAYLOAD);
  w.f32(rcl.roll);                     //0  roll stick [-1..1]
  w.f32(rcl.pitch);                    //4  pitch stick [-1..1]
  w.f32(rcl.yaw);                      //8  yaw stick [-1..1]
  w.f32(rcl.throttle);                 //12 throttle stick [0..1]
  w.f32(rcl.vspeed);                   //16 vertical speed stick [-1..1]
  w.u8(rcl.armed ? 1 : 0);             //20 armed
  w.u8(rcl.flightmode_idx);            //21 flight mode index
  w.u8(0);                             //22 reserved
  w.u8(0);                             //23 reserved
  for(uint8_t i = 0; i < 8; i++) {     //24 channel 1..8 pwm [us]
    w.u16(rcl.pwm[i]);
  }
  send(TEL_MSG_RC, _payload, w.size());
}

void Telemetry::send_battery() {
  ByteWriter w(_payload, TEL_MAX_PAYLOAD);
  w.f32(bat.v);                        //0 voltage [V]
  w.f32(bat.i);                        //4 current [A]
  w.f32(bat.w);                        //8 power [W]
  w.f32(bat.mah);                      //12 used capacity [mAh]
  w.f32(bat.wh);                       //16 used capacity [Wh]
  send(TEL_MSG_BATTERY, _payload, w.size());
}

void Telemetry::send_motors() {
  ByteWriter w(_payload, TEL_MAX_PAYLOAD);
  w.u8(out.armed() ? 1 : 0);           //0 armed
  w.u8((uint8_t)out.mode());           //1 output mode
  w.u8(4);                             //2 number of motors
  w.u8(0);                             //3 reserved
  for(uint8_t i = 0; i < 8; i++) {     //4 motor commands [0..1]
    w.f32(out.command[i]);
  }
  send(TEL_MSG_MOTORS, _payload, w.size());
}

void Telemetry::send_pid() {
  ByteWriter w(_payload, TEL_MAX_PAYLOAD);
  w.f32(pid.roll.setpoint);            //0
  w.f32(pid.roll.actual);              //4
  w.f32(pid.roll.sum);                 //8
  w.f32(pid.pitch.setpoint);           //12
  w.f32(pid.pitch.actual);             //16
  w.f32(pid.pitch.sum);                //20
  w.f32(pid.yaw.setpoint);             //24
  w.f32(pid.yaw.actual);               //28
  w.f32(pid.yaw.sum);                  //32
  w.f32(pid.throttle.setpoint);        //36
  w.f32(pid.throttle.actual);          //40
  w.f32(pid.throttle.sum);             //44
  send(TEL_MSG_PID, _payload, w.size());
}
