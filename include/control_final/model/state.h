#pragma once
#include "raytracer/data_structures/object_vector.h"

#include <array>

namespace control_final {

struct Pose {
  double get_x() { return _pose[0]; }
  double get_y() { return _pose[1]; }
  double get_z() { return _pose[2]; }

  double get_xdot() { return _pose[3]; }
  double get_ydot() { return _pose[4]; }
  double get_zdot() { return _pose[5]; }

  double get_theta_x() { return _pose[6]; }
  double get_theta_y() { return _pose[7]; }
  double get_theta_z() { return _pose[8]; }

  double get_theta_dot_x() { return _pose[9]; }
  double get_theta_dot_y() { return _pose[10]; }
  double get_theta_dot_z() { return _pose[11]; }

  double set_x(double x) { return _pose[0] = x; }
  double set_y(double y) { return _pose[1] = y; }
  double set_z(double z) { return _pose[2] = z; }

  double set_xdot(double xdot) { return _pose[3] = xdot; }
  double set_ydot(double ydot) { return _pose[4] = ydot; }
  double set_zdot(double zdot) { return _pose[5] = zdot; }

  double set_theta_x(double theta_x) { return _pose[6] = theta_x; }
  double set_theta_y(double theta_y) { return _pose[7] = theta_y; }
  double set_theta_z(double theta_z) { return _pose[8] = theta_z; }

  double set_theta_dot_x(double theta_dot_x) { return _pose[9] = theta_dot_x; }
  double set_theta_dot_y(double theta_dot_y) { return _pose[10] = theta_dot_y; }
  double set_theta_dot_z(double theta_dot_z) { return _pose[11] = theta_dot_z; }

private:
  std::array<double, 12> _pose;
};

struct State {
  Pose ball_pose;
  Pose table_pose;
  Pose camera_pose;

  raytracer::ObjectVector to_object_vector() const;
};

} // namespace control_final
