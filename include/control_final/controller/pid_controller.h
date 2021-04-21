#pragma once
#include "control_final/controller/controller.h"
#include "control_final/sensor/sensor.h"

#include "yaml-cpp/yaml.h"

namespace control_final {

class PIDController : public Controller {
public:
  PIDController(const YAML::Node &node);
  void react(std::vector<char> &pixs, const Sensor &sensor,
             const double t) override;

  void step(ControllerOutput &u, TablePose table_pose) override;

private:
  // this are the refrences for the outer loop
  double m_ref_x;
  double m_ref_y;

  // this are the refrences for the inner loop
  double m_ref_theta_x;
  double m_ref_theta_y;

  // gains
  double m_Kp;
  double m_Kd;
  double m_Ki;
  double m_inner_Kp;
  double m_inner_Kd;

  double m_int_err_x;
  double m_int_err_y;

  // TODO: this should be read from config
  double m_dt;
};
} // namespace control_final
