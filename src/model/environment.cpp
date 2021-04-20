#include "control_final/model/environment.h"
#include "control_final/controller/controller.h"
#include "control_final/model/render_config.h"

#include "raytracer/data_structures/object_vector.h"

#include "yaml.h"
#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <math.h>
#include <memory>
#include <vector>

namespace control_final {

Environment::Environment(const YAML::Node &node) {
  auto env_node = node["environment"];

  // error checking
  if (!env_node["ball_mass"]) {
    std::cout << "Did not specify ball_mass in environment node" << std::endl;
    exit(1);
  } else if (!env_node["ball_radius"]) {
    std::cout << "Did not specify ball_radius in environment node" << std::endl;
    exit(1);
  } else if (!env_node["table_mass"]) {
    std::cout << "Did not specify table_mass in environment node" << std::endl;
    exit(1);
  } else if (!env_node["table_height"]) {
    std::cout << "Did not specify table_height in environment node"
              << std::endl;
    exit(1);
  } else if (!env_node["table_radius"]) {
    std::cout << "Did not specify table_radius in environment node"
              << std::endl;
    exit(1);
  } else if (!env_node["dt"]) {
    std::cout << "Did not specify dt in environment node" << std::endl;
    exit(1);
  } else if (!env_node["mu"]) {
    std::cout << "Did not specify mu in environment node" << std::endl;
    exit(1);
  }

  ball_mass = env_node["ball_mass"].as<double>();
  ball_radius = env_node["ball_radius"].as<double>();
  table_height = env_node["table_height"].as<double>();
  table_mass = env_node["table_mass"].as<double>();
  table_radius = env_node["table_radius"].as<double>();
  dt = env_node["dt"].as<double>();
  mu = env_node["mu"].as<double>();

  // set state to be all zeros (expect axis of rotation which wouldn't make
  // sense)
  const Eigen::Vector3d init_aor{0, 1, 0};
  BallPose ball_pose{0, 0, 0, 0, 0, 0, init_aor, 0};
  TablePose table_pose{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  m_state = {ball_pose, table_pose};
}

void Environment::step(const ControllerOutput &u) {
  apply_controller_output(u);

  // check if ball is in contact with the table and apply corrective force
  const Eigen::Vector3d normal_vec = get_table_normal_vec();
  const Eigen::Vector3d table_to_ball = get_ball_pos() - get_table_pos();
  const double dist_from_table = (table_to_ball).dot(normal_vec);
  const double dist_from_center =
      (table_to_ball - dist_from_table * normal_vec).norm();

  if (dist_from_table < ball_radius && dist_from_center < table_radius) {
    // Resolve colision
    const double intersection_depth = dist_from_table - ball_radius;
    Eigen::Vector3d delta_v = -intersection_depth * normal_vec / dt;

    // this is an approximation for losing energy during a bounce
    // TODO: energy lost should be in config
    double extra_v = (get_ball_vel() + delta_v).dot(normal_vec);
    if (extra_v > 0) {
      delta_v -= 0.75 * extra_v * delta_v;
    }

    // handle friction
    // Calculate force that we just exerted (this is the normal force)
    const Eigen::Vector3d normal_force = delta_v * ball_mass / dt;

    apply_force(normal_force);

    // friction direction is the opposite of the ball's velocity projected onto
    // the table (note that this velocity is at the tangent point not the COM)
    Eigen::Vector3d neg_curr_vel =
        -get_ball_vel() -
        get_ball_omega() * normal_vec.cross(get_ball_aor()).normalized();

    // project velocity onto table
    const Eigen::Vector3d friction_dir =
        (neg_curr_vel - neg_curr_vel.dot(normal_vec) * normal_vec).normalized();

    Eigen::Vector3d friction_force = friction_dir * normal_force.norm() * mu;

    const double curr_speed_on_table =
        (-(neg_curr_vel - neg_curr_vel.dot(normal_vec) * normal_vec)).norm();

    // Force shouldn't be stronger than enough to stop the ball
    const double dv = (friction_force * dt / ball_mass).norm();
    if (dv > curr_speed_on_table) {
      friction_force *= curr_speed_on_table / dv;
    }

    // calulate frictions affect of COM
    apply_force(friction_force);

    apply_torque(friction_force, normal_vec);
  }

  // find all of the forces acting on the ball
  Eigen::Vector3d gravity;
  gravity << 0, 0, -9.8;
  gravity *= ball_mass;

  apply_force(gravity);
  move_ball();
  move_table();
}

void Environment::apply_force(const Eigen::Vector3d &force) {
  m_state.ball_pose.xdot += dt * force[0] / ball_mass;
  m_state.ball_pose.ydot += dt * force[1] / ball_mass;
  m_state.ball_pose.zdot += dt * force[2] / ball_mass;
}

void Environment::apply_torque(const Eigen::Vector3d &force,
                               const Eigen::Vector3d &normal) {

  // First we calculate affect on omega
  // First we need torque vector
  const Eigen::Vector3d torque = (-normal * ball_radius).cross(force);
  const Eigen::Vector3d aor = get_ball_aor();

  const double omega = get_ball_omega();
  if (omega > 0) {
    set_ball_omega(get_ball_omega() + dt * torque.dot(aor) / get_ball_I());
  } else {
    set_ball_omega(get_ball_omega() + dt * torque.norm() / get_ball_I());
    set_ball_aor(torque.normalized());
  }

  // Next we calculate procession effects
  const Eigen::Vector3d torque_along_aor = torque.dot(aor) * aor;
  const Eigen::Vector3d torque_perp_to_aor = torque - torque_along_aor;

  const double L = get_angular_momentum();
  // This check is to avoid divide by zero errors when we aren't spinning
  // Besided if there is no angular momentum, there is no procession
  if (L) {
    Eigen::Vector3d d_aor = (1 / L) * torque_perp_to_aor * dt;
    set_ball_aor(aor + d_aor);
  }
}

void Environment::apply_controller_output(const ControllerOutput &u) {
  m_state.table_pose.theta_dot_x += dt * u.torque_x / get_table_I();
  m_state.table_pose.theta_dot_y += dt * u.torque_y / get_table_I();

  m_state.table_pose.theta_dot_x =
      std::max(-0.4, m_state.table_pose.theta_dot_x);
  m_state.table_pose.theta_dot_x =
      std::min(0.4, m_state.table_pose.theta_dot_x);
  m_state.table_pose.theta_dot_y =
      std::max(-0.4, m_state.table_pose.theta_dot_y);
  m_state.table_pose.theta_dot_y =
      std::min(0.4, m_state.table_pose.theta_dot_y);
}

void Environment::move_ball() {
  m_state.ball_pose.x += dt * m_state.ball_pose.xdot;
  m_state.ball_pose.y += dt * m_state.ball_pose.ydot;
  m_state.ball_pose.z += dt * m_state.ball_pose.zdot;
}
void Environment::move_table() {
  m_state.table_pose.x += dt * m_state.table_pose.xdot;
  m_state.table_pose.y += dt * m_state.table_pose.ydot;
  m_state.table_pose.z += dt * m_state.table_pose.zdot;

  m_state.table_pose.theta_x += dt * m_state.table_pose.theta_dot_x;
  m_state.table_pose.theta_y += dt * m_state.table_pose.theta_dot_y;
}

void Environment::apply_vel(const Eigen::Vector3d &vel) {
  m_state.ball_pose.x += dt * vel[0];
  m_state.ball_pose.y += dt * vel[1];
  m_state.ball_pose.z += dt * vel[2];
}

Eigen::Vector3d Environment::get_table_normal_vec() const {
  Eigen::Vector3d normal_vector;
  normal_vector[0] = sin(m_state.table_pose.theta_y);
  normal_vector[1] = sin(m_state.table_pose.theta_x);
  normal_vector[2] = sqrt(1 - (pow(sin(m_state.table_pose.theta_x), 2) +
                               pow(sin(m_state.table_pose.theta_y), 2)));
  return normal_vector;
}

} // namespace control_final
