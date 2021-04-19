#pragma once
#include "control_final/controller/controller.h"
#include "control_final/controller/reference.h"
#include "control_final/sensor/sensor.h"

namespace control_final {

class PIDController : public Controller {
public:
  PIDController(const std::string &filename);
  void react(std::vector<char> &pixs, Reference &u, const Sensor &sensor,
             const double t) override;



private:
  // TODO: right now these are used to throttle the controller but I want to 
  // switch to a force based model
  double m_last_x;
  double m_last_y;

  double m_ref_x;
  double m_ref_y;

  // gains
  double m_Kp;
  double m_Kd;
  double m_Ki;

  double m_int_err_x;
  double m_int_err_y;

  // TODO: this should be read from config
  static constexpr double m_dt = 0.001;
};
} // namespace control_final
