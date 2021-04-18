#pragma once
#include "control_final/controller/reference.h"
#include "control_final/controller/controller.h"

namespace control_final {

class PIDController : public Controller {
public:
  PIDController(const std::string &filename);
  void react(std::vector<char> &pixs, Reference &u);
};
} // namespace control_final
