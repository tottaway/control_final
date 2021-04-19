#pragma once
#include "control_final/controller/controller.h"
#include "control_final/controller/reference.h"

namespace control_final {

class PIDController : public Controller {
public:
  PIDController(const std::string &filename);
  void react(std::vector<char> &pixs, Reference &u, const Sensor &sensor);

private:
  double m_last_x;
  double m_last_y;
};
} // namespace control_final
