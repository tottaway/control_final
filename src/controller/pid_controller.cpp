#include "control_final/controller/controller.h"
#include "control_final/controller/pid_controller.h"

#include "yaml.h"

namespace control_final {
PIDController::PIDController(const std::string &filename)
    : Controller(filename) {
  // TODO: figure how I want to configure PID controller
}

void PIDController::react(std::vector<char> &pixs, Reference &u) {
  u.table_pose.x = 0;
  u.table_pose.y = 0;
  u.table_pose.z = 0;
  u.table_pose.xdot = 0;
  u.table_pose.ydot = 0;
  u.table_pose.zdot = 0;
  u.table_pose.theta_x = 0;
  u.table_pose.theta_y = 0;
  u.table_pose.theta_dot_x = 0;
  u.table_pose.theta_dot_y = 0;
}

} // namespace control_final
