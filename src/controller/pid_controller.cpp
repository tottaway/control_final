#include "control_final/controller/pid_controller.h"

namespace control_final {
void PIDController::react(std::vector<char> &pixs, Reference &u) {
  _predict_state(pixs);
}
}
