#pragma once
#include "raytracer/data_structures/object_vector.h"

#include <array>

namespace control_final {

struct Pose {
  double x;
  double y;
  double z;

  double xdot;
  double ydot;
  double zdot;

  double theta_x;
  double theta_y;
  double theta_z;

  double theta_dot_x;
  double theta_dot_y;
  double theta_dot_z;
};

struct State {
  Pose ball_pose;
  Pose table_pose;
  Pose camera_pose;

  raytracer::ObjectVector to_object_vector() const;
};

} // namespace control_final
