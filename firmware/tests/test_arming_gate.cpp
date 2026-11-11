#include "../src/arming_gate.h"
#include <cassert>

int main() {
  ArmingGate gate;
  assert(!gate.update(false, false));
  assert(!gate.update(true, true)); // first sighting armed is not an arm edge
  assert(!gate.update(true, false));
  assert(gate.update(true, true));
  assert(!gate.update(false, true)); // disarm immediately on loss
  assert(!gate.update(false, true)); // no oscillation while disconnected
  assert(!gate.update(true, true));  // reconnect cannot re-arm
  assert(!gate.update(true, false)); // explicit disarm clears inhibit
  assert(gate.update(true, true));
  assert(!gate.update(true, false)); // kill switch
}
