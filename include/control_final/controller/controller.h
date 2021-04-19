#pragma once
#include "control_final/controller/reference.h"
#include "control_final/sensor/sensor.h"

#include "yaml.h"
#include <deque>

namespace control_final {

class Controller {
public:
  Controller(const YAML::Node &node);
  // we take in the sensor so that we can have access to parameters like the
  // camera location and dir
  virtual void react(std::vector<char> &pixs, Reference &u,
                     const Sensor &sensor, const double t) = 0;

  double get_curr_x() {return m_history[0].ball_pose.x;}
  double get_curr_y() {return m_history[0].ball_pose.y;}
  double get_prev_x() {return m_history[1].ball_pose.x;}
  double get_prev_y() {return m_history[1].ball_pose.y;}

protected:
  void predict_state(std::vector<char> &pixs, const Sensor &sensor);

  unsigned m_history_len;
  std::deque<State> m_history;
};

} // namespace control_final
