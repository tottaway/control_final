#include "control_final/model/environment.h"

#include <Eigen/Dense>
#include <iostream>

namespace control_final {

void Environment::step(const Reference &u) {
  set_table_pose(u.table_pose);

  // find all of the forces acting on the ball
  Eigen::Vector3d gravity;
  gravity << 0, 0, -9.8;
  gravity *= BALL_MASS;

  _apply_force(gravity);
  _move_ball();

  // assest that ball is in contact with the table
  const Eigen::Vector3d normal_vec = get_table_normal_vec();
  const Eigen::Vector3d table_to_ball = get_ball_pos() - get_table_pos();
  const double dist_from_table = (table_to_ball).dot(normal_vec);
  const double dist_from_center =
      (table_to_ball - dist_from_table * normal_vec).norm();

  if (dist_from_table < BALL_RADIUS && dist_from_center < TABLE_RADIUS) {
    // Resolve colision
    const double intersection_depth = dist_from_table - BALL_RADIUS;
    Eigen::Vector3d delta_v = -intersection_depth * normal_vec / DT;

    // This makes sure that the ball is tagent to the table at the end of every
    // timestep

    double extra_v = (get_ball_vel() - delta_v).dot(normal_vec);
    if (extra_v > 0) {
      delta_v -= 0.8 * extra_v * delta_v;
    }

    set_ball_vel(get_ball_vel() + delta_v);

    // handle friction
    // Calculate force that we just exerted (this is the normal force)
    const Eigen::Vector3d normal_force = delta_v * BALL_MASS / DT;
    const double mu = 0.6;

    // friction direction is the opposite of the ball's velocity projected onto
    // the table (note that this velocity is at the tangent point not the COM
    Eigen::Vector3d neg_curr_vel = -get_ball_vel();
    neg_curr_vel -= get_ball_omega() *
      normal_vec.cross(get_ball_aor()).normalized();

    // project velocity onto table
    const Eigen::Vector3d friction_dir =
        (neg_curr_vel - neg_curr_vel.dot(normal_vec) * normal_vec).normalized();

    const Eigen::Vector3d friction_force = friction_dir * normal_force.norm() * mu;

    // calulate frictions affect of COM
    _apply_force(friction_force);

    _apply_torque(friction_force, normal_vec);
  }
}

void Environment::_apply_force(const Eigen::Vector3d &force) {
  _state.ball_pose.xdot += DT * force[0] / BALL_MASS;
  _state.ball_pose.ydot += DT * force[1] / BALL_MASS;
  _state.ball_pose.zdot += DT * force[2] / BALL_MASS;
}

void Environment::_apply_torque(const Eigen::Vector3d &force,
                                const Eigen::Vector3d &normal) {

  // First we calculate affect on omega
  // First we need torque vector
  const Eigen::Vector3d torque = (-normal * BALL_RADIUS).cross(force);
  const Eigen::Vector3d aor = get_ball_aor();

  set_ball_omega(get_ball_omega() + DT * torque.dot(aor) / I);

  // Next we calculate procession effects
  const Eigen::Vector3d torque_along_aor = torque.dot(aor) * aor;
  const Eigen::Vector3d torque_perp_to_aor = torque - torque_along_aor;
  Eigen::Vector3d d_aor = (1 / get_angular_momentum()) * torque_perp_to_aor * DT;
  set_ball_aor(aor + d_aor);

  
}

void Environment::_move_ball() {
  _state.ball_pose.x += DT * _state.ball_pose.xdot;
  _state.ball_pose.y += DT * _state.ball_pose.ydot;
  _state.ball_pose.z += DT * _state.ball_pose.zdot;
}

void Environment::_apply_vel(const Eigen::Vector3d &vel) {
  _state.ball_pose.x += DT * vel[0];
  _state.ball_pose.y += DT * vel[1];
  _state.ball_pose.z += DT * vel[2];
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
