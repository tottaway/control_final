#include "control_final/controller/pid_controller.h"
#include "control_final/controller/controller.h"
#include "control_final/sensor/sensor.h"

#include "yaml.h"
#include <algorithm>
#include <iostream>

namespace control_final {
PIDController::PIDController(const YAML::Node &node) : Controller(node) {

  // TODO: make this better
  auto controller_node = node["controller"];
  m_Kp = controller_node["Kp"].as<double>();
  m_Kd = controller_node["Kd"].as<double>();
  m_Ki = controller_node["Ki"].as<double>();

  m_ref_x = controller_node["ref"][0].as<double>();
  m_ref_y = controller_node["ref"][1].as<double>();

  m_last_x = 0;
  m_last_y = 0;
}

void PIDController::react(std::vector<char> &pixs, Reference &u,
                          const Sensor &sensor, const double t) {
  // These fields never change
  u.table_pose.x = 0;
  u.table_pose.y = 0;
  u.table_pose.z = 0;
  u.table_pose.xdot = 0;
  u.table_pose.ydot = 0;
  u.table_pose.zdot = 0;

  // This handles the CV part
  Controller::predict_state(pixs, sensor);

  // get the current and previous states
  const double curr_x = Controller::get_curr_x();
  const double curr_y = Controller::get_curr_y();
  const double prev_x = Controller::get_prev_x();
  const double prev_y = Controller::get_prev_y();

  const double curr_err_x = m_ref_x - curr_x;
  const double curr_err_y = m_ref_y - curr_y;
  const double prev_err_x = m_ref_x - prev_x;
  const double prev_err_y = m_ref_y - prev_y;

  const double d_err_x = (curr_err_x - prev_err_x) / m_dt;
  const double d_err_y = (curr_err_y - prev_err_y) / m_dt;
  std::cout << d_err_x << std::endl;

  m_int_err_x += curr_err_x * m_dt;
  m_int_err_y += curr_err_y * m_dt;

  // Note that rotating around the x axis affects the y coord of the ball
  // TODO: implement windup
  const double control_x =
      m_Kp * curr_err_y + m_Kd * d_err_y + m_Ki * m_int_err_y;
  const double control_y =
      m_Kp * curr_err_x + m_Kd * d_err_x + m_Ki * m_int_err_x;

  // TODO: get dt
  u.table_pose.theta_dot_x = (control_x - m_last_x) * 5;
  u.table_pose.theta_dot_y = (control_y - m_last_y) * 5;

  m_last_x = m_last_x + ((control_x - m_last_x) / 200);
  m_last_y = m_last_y + ((control_y - m_last_y) / 200);
}

} // namespace control_final
