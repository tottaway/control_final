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

  m_inner_Kp = controller_node["inner_Kp"].as<double>();
  m_inner_Kd = controller_node["inner_Kd"].as<double>();

  m_ref_x = controller_node["ref"][0].as<double>();
  m_ref_y = controller_node["ref"][1].as<double>();

  m_ref_theta_x = 0;
  m_ref_theta_y = 0;

  m_int_err_x = 0;
  m_int_err_y = 0;
}

void PIDController::react(std::vector<char> &pixs, const Sensor &sensor,
                          const double t) {
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

  m_int_err_x += curr_err_x * m_dt;
  m_int_err_y += curr_err_y * m_dt;

  // Note that rotating around the x axis affects the y coord of the ball
  // TODO: implement windup
  m_ref_theta_x = m_Kp * curr_err_y + m_Kd * d_err_y + m_Ki * m_int_err_y;
  m_ref_theta_y = m_Kp * curr_err_x + m_Kd * d_err_x + m_Ki * m_int_err_x;

  // Handle saturation
  m_ref_theta_x = std::max(-0.523, m_ref_theta_x);
  m_ref_theta_x = std::min(0.523, m_ref_theta_x);
  m_ref_theta_y = std::max(-0.523, m_ref_theta_y);
  m_ref_theta_y = std::min(0.523, m_ref_theta_y);


}

void PIDController::step(ControllerOutput &u, TablePose table_pose) {

  // get the current and previous states
  const double curr_theta_x = table_pose.theta_x;
  const double curr_theta_y = table_pose.theta_y;

  const double curr_err_x = m_ref_theta_x - curr_theta_x;
  const double curr_err_y = m_ref_theta_y - curr_theta_y;

  const int curr_err_x_sign = (curr_err_x > 0) - (curr_err_x < 0);
  const int curr_err_y_sign = (curr_err_y > 0) - (curr_err_y < 0);
  const double d_err_x = -curr_err_x_sign * table_pose.theta_dot_x;
  const double d_err_y = -curr_err_y_sign * table_pose.theta_dot_y;

  // Note that rotating around the x axis affects the y coord of the ball
  // TODO: implement windup
  u.torque_x = m_inner_Kp * curr_err_x + m_inner_Kd * d_err_x;
  u.torque_y = m_inner_Kp * curr_err_y + m_inner_Kd * d_err_y;
}

} // namespace control_final
