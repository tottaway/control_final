#pragma once
#include "control_final/controller/reference.h"
#include "control_final/model/state.h"

#include "raytracer/data_structures/object_vector.h"

#include "yaml.h"
#include <string>

namespace control_final {

class Environment {
public:
  // Constants for the scene objects
  // these are read from config/env_config.yaml
  double ball_radius;
  double ball_mass;
  double table_radius;
  double table_height;
  double dt;
  double mu;

  // Constructor initializes state to all zeros
  Environment() {
    const Eigen::Vector3d init_aor{0, 1, 0};
    BallPose ball_pose{0, 0, 0, 0, 0, 0, init_aor, 0};
    TablePose table_pose{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    m_state = {ball_pose, table_pose};
  };

  Environment(const YAML::Node &node);

  // advances scene by dt
  void step(const Reference &u);

  // Lots of getters
  State get_state() const { return m_state; };
  BallPose get_ball_pose() const { return m_state.ball_pose; };
  TablePose get_table_pose() const { return m_state.table_pose; };

  Eigen::Vector3d get_table_pos() const {
    Eigen::Vector3d res;
    res << m_state.table_pose.x, m_state.table_pose.y, m_state.table_pose.z;
    return res;
  };

  Eigen::Vector3d get_table_vel() const {
    Eigen::Vector3d res;
    res << m_state.table_pose.xdot, m_state.table_pose.ydot,
        m_state.table_pose.zdot;
    return res;
  };

  Eigen::Vector3d get_ball_pos() const {
    Eigen::Vector3d res;
    res << m_state.ball_pose.x, m_state.ball_pose.y, m_state.ball_pose.z;
    return res;
  };

  Eigen::Vector3d get_ball_vel() const {
    Eigen::Vector3d res;
    res << m_state.ball_pose.xdot, m_state.ball_pose.ydot,
        m_state.ball_pose.zdot;
    return res;
  };

  // aor == axis_of_rotation
  Eigen::Vector3d get_ball_aor() const {
    return m_state.ball_pose.axis_of_rotation;
  }

  double get_ball_omega() const { return m_state.ball_pose.omega; }

  // Assumes that the ball is a shell
  double get_angular_momentum() const {
    return get_I() * m_state.ball_pose.omega;
  }

  Eigen::Vector3d get_table_normal_vec() const;
  double get_I() const {
    return (2. / 3.) * ball_mass * ball_radius * ball_radius;
  }

  // Lots of setters
  void set_table_pose(const TablePose new_pose) {
    m_state.table_pose = new_pose;
  }
  void set_ball_pose(const BallPose new_pose) {
    m_state.ball_pose = new_pose;
    m_state.ball_pose.axis_of_rotation.normalize();
  }
  void set_table_pos(const Eigen::Vector3d new_pos) {
    m_state.table_pose.x = new_pos[0];
    m_state.table_pose.y = new_pos[1];
    m_state.table_pose.z = new_pos[2];
  };
  void set_table_vel(const Eigen::Vector3d new_vel) {
    m_state.table_pose.xdot = new_vel[0];
    m_state.table_pose.ydot = new_vel[1];
    m_state.table_pose.zdot = new_vel[2];
  };
  void set_ball_pos(const Eigen::Vector3d new_pos) {
    m_state.ball_pose.x = new_pos[0];
    m_state.ball_pose.y = new_pos[1];
    m_state.ball_pose.z = new_pos[2];
  };
  void set_ball_vel(const Eigen::Vector3d new_vel) {
    m_state.ball_pose.xdot = new_vel[0];
    m_state.ball_pose.ydot = new_vel[1];
    m_state.ball_pose.zdot = new_vel[2];
  };

  // aor == axis_of_rotation,
  // also enforces that aor is normalized
  void set_ball_aor(const Eigen::Vector3d new_aor) {
    m_state.ball_pose.axis_of_rotation = new_aor.normalized();
  };

  void set_ball_omega(const double new_omega) {
    m_state.ball_pose.omega = new_omega;
  };

private:
  State m_state;

  // updates ball velocity based on forces
  void apply_force(const Eigen::Vector3d &force);

  // update ball's axis of rotation and angular velocity based on force which
  // is applied to the surface of the sphere where normal is the normalized
  // vector between the point the force is acting and the center of the ball
  void apply_torque(const Eigen::Vector3d &force,
                    const Eigen::Vector3d &normal);

  // moves the ball according to the current velocity
  void move_ball();

  // moves the table according to the current velocity
  void move_table();

  // moves the ball according to passed in velocity
  void apply_vel(const Eigen::Vector3d &vel);
};

} // namespace control_final
