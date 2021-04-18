#pragma once
#include "control_final/controller/reference.h"
#include "control_final/model/state.h"

#include "raytracer/data_structures/object_vector.h"

namespace control_final {

class Environment {
public:
  // Constants for the scene objects
  // TODO: read these from config file
  constexpr static double BALL_RADIUS = 0.04;
  constexpr static double BALL_MASS = 0.027;
  constexpr static double TABLE_RADIUS = 0.4;
  constexpr static double TABLE_HEIGHT = 0.01;
  constexpr static double DT = 1. / 1000;
  constexpr static double I = (2. / 3.) * BALL_MASS * BALL_RADIUS * BALL_RADIUS;

  constexpr static bool ROTATION_VIZ = false;

  // Constructor initializes state to all zeros
  Environment() {
    const Eigen::Vector3d init_aor{0, 1, 0};
    BallPose ball_pose{0, 0, 0, 0, 0, 0, init_aor, 0};
    TablePose table_pose{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    _state = {ball_pose, table_pose};
  };

  // advances scene by dt
  void step(const Reference &u);

  // Lots of getters
  State get_state() const { return _state; };
  Eigen::Vector3d get_table_pos() const {
    Eigen::Vector3d res;
    res << _state.table_pose.x, _state.table_pose.y, _state.table_pose.z;
    return res;
  };
  Eigen::Vector3d get_table_vel() const {
    Eigen::Vector3d res;
    res << _state.table_pose.xdot, _state.table_pose.ydot,
        _state.table_pose.zdot;
    return res;
  };
  Eigen::Vector3d get_ball_pos() const {
    Eigen::Vector3d res;
    res << _state.ball_pose.x, _state.ball_pose.y, _state.ball_pose.z;
    return res;
  };
  Eigen::Vector3d get_ball_vel() const {
    Eigen::Vector3d res;
    res << _state.ball_pose.xdot, _state.ball_pose.ydot, _state.ball_pose.zdot;
    return res;
  };

  // aor == axis_of_rotation
  Eigen::Vector3d get_ball_aor() const { return _state.ball_pose.axis_of_rotation; }

  double get_ball_omega() const { return _state.ball_pose.omega; }

  // Assumes that the ball is a shell
  double get_angular_momentum() const { return I * _state.ball_pose.omega; }

  Eigen::Vector3d get_table_normal_vec() const;

  // Lots of setters
  void set_table_pose(const TablePose new_pose) {
    _state.table_pose = new_pose;
  }
  void set_ball_pose(const BallPose new_pose) {
    _state.ball_pose = new_pose;
    _state.ball_pose.axis_of_rotation.normalize();
  }
  void set_table_pos(const Eigen::Vector3d new_pos) {
    _state.table_pose.x = new_pos[0];
    _state.table_pose.y = new_pos[1];
    _state.table_pose.z = new_pos[2];
  };
  void set_table_vel(const Eigen::Vector3d new_vel) {
    _state.table_pose.xdot = new_vel[0];
    _state.table_pose.ydot = new_vel[1];
    _state.table_pose.zdot = new_vel[2];
  };
  void set_ball_pos(const Eigen::Vector3d new_pos) {
    _state.ball_pose.x = new_pos[0];
    _state.ball_pose.y = new_pos[1];
    _state.ball_pose.z = new_pos[2];
  };
  void set_ball_vel(const Eigen::Vector3d new_vel) {
    _state.ball_pose.xdot = new_vel[0];
    _state.ball_pose.ydot = new_vel[1];
    _state.ball_pose.zdot = new_vel[2];
  };

  // aor == axis_of_rotation,
  // also enforces that aor is normalized
  void set_ball_aor(const Eigen::Vector3d new_aor) {
    _state.ball_pose.axis_of_rotation = new_aor.normalized();
  };

  void set_ball_omega(const double new_omega) {
    _state.ball_pose.omega = new_omega;
  };

  raytracer::ObjectVector to_object_vector() const;

private:
  State _state;

  // updates ball velocity based on forces
  void _apply_force(const Eigen::Vector3d &force);

  // update ball's axis of rotation and angular velocity based on force which
  // is applied to the surface of the sphere where normal is the normalized
  // vector between the point the force is acting and the center of the ball
  void _apply_torque(const Eigen::Vector3d &force,
                     const Eigen::Vector3d &normal);

  // moves the ball according to the current velocity
  void _move_ball();

  // moves the ball according to passed in velocity
  void _apply_vel(const Eigen::Vector3d &vel);
};

} // namespace control_final
