#pragma once
#include "control_final/sensor/sensor.h"

#include "yaml-cpp/yaml.h"
#include <deque>

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
  virtual void step(ControllerOutput &u, TablePose table_pose) = 0;

  double get_curr_x() { return m_history[0].ball_pose.x; }
  double get_curr_y() { return m_history[0].ball_pose.y; }
  double get_prev_x() { return m_history[1].ball_pose.x; }
  double get_prev_y() { return m_history[1].ball_pose.y; }

protected:
  void predict_state(std::vector<char> &pixs, const Sensor &sensor);

  unsigned m_history_len;
  std::deque<State> m_history;
};

} // namespace control_final
