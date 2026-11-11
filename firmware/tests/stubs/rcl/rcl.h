#pragma once
#include <cstdint>
#include <cstring>
#include <algorithm>
#define RCL_MAX_CH 20
#define RCL_THROTTLE_DEADBAND 60
template<class T> T constrain(T v, T lo, T hi) { return std::max(lo, std::min(hi, v)); }
class RclGizmo { public: virtual ~RclGizmo() {} virtual bool update() = 0; };
struct FakeConfig {
  int rcl_rol_ch=1, rcl_rol_left=1100, rcl_rol_mid=1500, rcl_rol_right=1900;
  int rcl_pit_ch=2, rcl_pit_pull=1100, rcl_pit_mid=1500, rcl_pit_push=1900;
  int rcl_yaw_ch=4, rcl_yaw_left=1100, rcl_yaw_mid=1500, rcl_yaw_right=1900;
  int rcl_thr_ch=3, rcl_thr_pull=1100, rcl_thr_push=1900;
  int rcl_flt_ch=6, rcl_flt_min=1165, rcl_flt_max=1815;
  int rcl_arm_ch=5, rcl_arm_min=1600, rcl_arm_max=2500;
};
extern FakeConfig cfg;
