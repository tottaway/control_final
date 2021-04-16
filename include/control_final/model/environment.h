#pragma once
#include "control_final/controller/reference.h"
#include "control_final/model/state.h"

namespace control_final {

class Environment {
public:
  // Constants for the scene objects
  // TODO: read these from config file
  constexpr static double BALL_RADIUS = 0.5;
  constexpr static double TABLE_RADIUS = 2;
  constexpr static double TABLE_HEIGHT = 0.05;
  constexpr static double DT = 1. / 32;

  // Constructor initializes state to all zeros
  Environment() {
    Pose zeros{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    _state = {zeros, zeros, zeros};
  };

  // advances scene by dt
  void step(const Reference &u);

  // Lots of getters
  State get_state() { return _state; };

  Eigen::Vector3d get_table_pos() {
    Eigen::Vector3d res;
    res << _state.table_pose.x, _state.table_pose.y, _state.table_pose.z;
    return res;
  };
  Eigen::Vector3d get_table_vel() {
    Eigen::Vector3d res;
    res << _state.table_pose.xdot, _state.table_pose.ydot,
        _state.table_pose.zdot;
    return res;
  };
  Eigen::Vector3d get_ball_pos() {
    Eigen::Vector3d res;
    res << _state.ball_pose.x, _state.ball_pose.y, _state.ball_pose.z;
    return res;
  };
  Eigen::Vector3d get_ball_vel() {
    Eigen::Vector3d res;
    res << _state.ball_pose.xdot, _state.ball_pose.ydot, _state.ball_pose.zdot;
    return res;
  };

  Eigen::Vector3d get_table_normal_vec();

  // Lots of setters
  void set_table_pose(const Pose new_pose) { _state.table_pose = new_pose; }
  void set_ball_pose(const Pose new_pose) { _state.ball_pose = new_pose; }
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

private:
  State _state;

  // updates ball velocity based on forces
  void _apply_force(const Eigen::Vector3d &force);

  // moves the ball according to the current velocity
  void _move_ball();
};

} // namespace control_final
