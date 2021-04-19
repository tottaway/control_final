#include "control_final/controller/pid_controller.h"
#include "control_final/controller/controller.h"
#include "control_final/sensor/sensor.h"

#include "yaml.h"
#include <iostream>

namespace control_final {
PIDController::PIDController(const std::string &filename)
    : Controller(filename) {
  // TODO: figure how I want to configure PID controller
  m_last_x = 0;
  m_last_y = 0;
}

void PIDController::react(std::vector<char> &pixs, Reference &u,
                          const Sensor &sensor) {
  State predicted_state = predict_state(pixs, sensor);
  std::cout << "[" << predicted_state.ball_pose.x << ", "
            << predicted_state.ball_pose.y << "]" << std::endl;

  // These fields never change
  u.table_pose.x = 0;
  u.table_pose.y = 0;
  u.table_pose.z = 0;
  u.table_pose.xdot = 0;
  u.table_pose.ydot = 0;
  u.table_pose.zdot = 0;

  const double goal_theta_x = 0.01 * predicted_state.ball_pose.y;
  const double goal_theta_y = 0.01 * predicted_state.ball_pose.x;

  // TODO: get dt
  u.table_pose.theta_dot_x = (goal_theta_x - m_last_x) * 100;
  u.table_pose.theta_dot_y = (goal_theta_x - m_last_y) * 100;

  m_last_x = goal_theta_x + ((goal_theta_x - m_last_x) / 10);
  m_last_y = goal_theta_y + ((goal_theta_y - m_last_y) / 10);
}

} // namespace control_final
