#include "control_final/model/environment.h"

#include <Eigen/Dense>
#include <iostream>

namespace control_final {

void Environment::step(const Reference &u) {
  // TODO: set table's pose to reference and then do object collision to move
  // ball
  set_table_pose(u.table_pose);

  // find all of the forces acting on the ball
  Eigen::Vector3d gravity;
  gravity << 0, 0, -9.8;

  // assest that ball is in contact with the table
  Eigen::Vector3d normal_vec = get_table_normal_vec();
  Eigen::Vector3d normal_force;
  double dist_from_table =
      (get_ball_pos() - get_table_pos()).dot(normal_vec) - BALL_RADIUS;

  // TODO: figure out if the ball is in the radius of the table

  if (abs(dist_from_table) > 1e-2) {
    std::cout << "Warning, Ball is not on the table" << std::endl;
    std::cout << "Distance is: " << dist_from_table << std::endl;
    set_ball_pos(get_ball_pos() - dist_from_table * normal_vec);
  }
  normal_force = gravity.dot(-normal_vec) * normal_vec;

  Eigen::Vector3d total_force = gravity + normal_force;

  _apply_force(total_force);
  _move_ball();
}

void Environment::_apply_force(const Eigen::Vector3d &force) {
  _state.ball_pose.xdot += DT * force[0];
  _state.ball_pose.ydot += DT * force[1];
  _state.ball_pose.zdot += DT * force[2];
}

void Environment::_move_ball() {
  _state.ball_pose.x += DT * _state.ball_pose.xdot;
  _state.ball_pose.y += DT * _state.ball_pose.ydot;
  _state.ball_pose.z += DT * _state.ball_pose.zdot;
}

Eigen::Vector3d Environment::get_table_normal_vec() {
  Eigen::Vector3d normal_vector;
  normal_vector[0] = sin(_state.table_pose.theta_y);
  normal_vector[1] = sin(_state.table_pose.theta_x);
  normal_vector[2] = sqrt(1 - (pow(sin(_state.table_pose.theta_x), 2) +
                               pow(sin(_state.table_pose.theta_y), 2)));
  return normal_vector;
}

} // namespace control_final
