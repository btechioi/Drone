#pragma once

// Require an observed disarmed state after every link loss before allowing a
// new arm. In particular, reconnection with a stale armed RC state is not an
// arming transition. This does not replace the receiver's own arm checks.
class ArmingGate {
 public:
  bool update(bool connected, bool receiver_armed) {
    if (!connected) {
      inhibit_ = true;
      return false;
    }
    if (!receiver_armed) {
      inhibit_ = false;
      return false;
    }
    return !inhibit_;
  }

 private:
  bool inhibit_ = true;
};
