#pragma once

#include <array>
#include <Eigen/Dense>

namespace control_final {

struct BallPose {
  double x;
  double y;
  double z;

  double xdot;
  double ydot;
  double zdot;

  // I only care about the angular velocity and what the axis of rotation is
  Eigen::Vector3d axis_of_rotation;
  double omega;
};

struct TablePose {
  double x;
  double y;
  double z;

  double xdot;
  double ydot;
  double zdot;

  double theta_x;
  double theta_y;

  double theta_dot_x;
  double theta_dot_y;
};

struct State {
  BallPose ball_pose;
  TablePose table_pose;
};

} // namespace control_final
