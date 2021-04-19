#pragma once
#include "control_final/controller/controller.h"
#include "control_final/controller/reference.h"

namespace control_final {

class PIDController : public Controller {
public:
  PIDController(const std::string &filename);
  void react(std::vector<char> &pixs, Reference &u, const Sensor &sensor);
};
} // namespace control_final
