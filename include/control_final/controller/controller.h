#pragma once
#include "control_final/sensor/sensor.h"

#include "kalman.hpp"
#include "yaml-cpp/yaml.h"
#include <deque>
#include <memory>

namespace control_final {

struct ControllerOutput {
  double torque_x;
  double torque_y;
};

class Controller {
public:
  Controller(const YAML::Node &node);
  // we take in the sensor so that we can have access to parameters like the
  // camera location and dir.

  // takes in image and sets the reference for the inner loop
  virtual void react(std::vector<char> &pixs, const Sensor &sensor,
                     const double t) = 0;

  // this sets the torques to apply to
  // TODO: maybe try to have this take in a noisy sensed table_pose
  virtual void step(ControllerOutput &u, TablePose table_pose);

  double get_curr_x() { return m_history[0][0]; }
  double get_curr_y() { return m_history[0][1]; }
  double get_prev_x() { return m_history[1][0]; }
  double get_prev_y() { return m_history[1][1]; }

  double get_ref_theta_x() { return m_ref_theta_x; }
  double get_ref_theta_y() { return m_ref_theta_y; }

protected:
  // this are the refrences for the inner loop
  double m_ref_theta_x;
  double m_ref_theta_y;

  // gains
  double m_inner_Kp;
  double m_inner_Kd;

  void predict_state(std::vector<char> &pixs, const Sensor &sensor);

  unsigned m_history_len;
  std::deque<Eigen::Vector4d> m_history;

  double m_dt;

  bool m_filter_initialized;
  std::unique_ptr<KalmanFilter> m_filter;
};

} // namespace control_final
