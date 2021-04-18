#include "control_final/model/environment.h"

#include "raytracer/data_structures/object_vector.h"
#include "raytracer/light.h"
#include "raytracer/material.h"
#include "raytracer/shapes.h"
#include "raytracer/util.h"

#include "yaml.h"
#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <memory>
#include <vector>

namespace control_final {} // namespace control_final

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

    // this is an approximation for losing energy during a bounce
    // TODO: energy lost should be in config
    double extra_v = (get_ball_vel() - delta_v).dot(normal_vec);
    if (extra_v > 0)
      delta_v -= 0.8 * extra_v * delta_v;

    set_ball_vel(get_ball_vel() + delta_v);

    // handle friction
    // Calculate force that we just exerted (this is the normal force)
    const Eigen::Vector3d normal_force = delta_v * BALL_MASS / DT;
    // TODO: mu should be in config
    constexpr double mu = 0.6;

    // friction direction is the opposite of the ball's velocity projected onto
    // the table (note that this velocity is at the tangent point not the COM)
    Eigen::Vector3d neg_curr_vel =
        -get_ball_vel() -
        get_ball_omega() * normal_vec.cross(get_ball_aor()).normalized();

    // project velocity onto table
    const Eigen::Vector3d friction_dir =
        (neg_curr_vel - neg_curr_vel.dot(normal_vec) * normal_vec).normalized();

    const Eigen::Vector3d friction_force =
        friction_dir * normal_force.norm() * mu;

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
  Eigen::Vector3d d_aor =
      (1 / get_angular_momentum()) * torque_perp_to_aor * DT;
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

Eigen::Vector3d Environment::get_table_normal_vec() const {
  Eigen::Vector3d normal_vector;
  normal_vector[0] = sin(_state.table_pose.theta_y);
  normal_vector[1] = sin(_state.table_pose.theta_x);
  normal_vector[2] = sqrt(1 - (pow(sin(_state.table_pose.theta_x), 2) +
                               pow(sin(_state.table_pose.theta_y), 2)));
  return normal_vector;
}

// Colors are BGR
// TODO: move to config
static const raytracer::Color ball_color{0, 165. / 255, 1};
static const raytracer::Color table_color{0.4, 0.4, 0.8};
static const raytracer::Color red{0, 0, 1};
static const raytracer::Color white{1, 1, 1};

static const raytracer::Material ball_material{.kd = 0.70,
                                               .ks = 0.10,
                                               .specular_exponent = 4,
                                               .diffuse_color = ball_color,
                                               .specular_color = white};

static const raytracer::Material table_material{.kd = 0.75,
                                                    .ks = 0.05,
                                                    .specular_exponent = 4,
                                                    .diffuse_color =
                                                        table_color,
                                                    .specular_color = white};

static const raytracer::Material rotation_viz_material{
    .kd = 0.75,
    .ks = 0.05,
    .specular_exponent = 4,
    .diffuse_color = red,
    .specular_color = white};

// and have sensor take in env and not state
raytracer::ObjectVector Environment::to_object_vector() const {
  std::vector<std::unique_ptr<raytracer::SceneObject>> objects;

  // First make the ball
  YAML::Node ball_node;

  const Eigen::Vector3d ball_pos = get_ball_pos();
  ball_node["primative"] = "sphere";
  ball_node["transforms"][0]["type"] = "scale";
  ball_node["transforms"][0]["scale_factor"] = BALL_RADIUS;

  // Note that since we've scaled everything we need to also scale the
  // translations
  ball_node["transforms"][1]["type"] = "translate";
  ball_node["transforms"][1]["dx"] = ball_pos[0] / BALL_RADIUS;
  ball_node["transforms"][1]["dy"] = ball_pos[1] / BALL_RADIUS;
  ball_node["transforms"][1]["dz"] = ball_pos[2] / BALL_RADIUS;

  // TODO: for some reason make_unique wasn't working so I'm doing this for
  // now
  objects.push_back(std::unique_ptr<raytracer::Sphere>(
      new raytracer::Sphere{ball_node, ball_material}));

  // Extra cylinder to visualize rotation
  if (ROTATION_VIZ) {
    YAML::Node rotation_viz;
    rotation_viz["primative"] = "cylinder";
    rotation_viz["transforms"][0]["type"] = "scale";
    rotation_viz["transforms"][0]["scale_factor"] = BALL_RADIUS;

    rotation_viz["transforms"][1]["type"] = "translate";
    rotation_viz["transforms"][1]["dx"] = ball_pos[0] / BALL_RADIUS;
    rotation_viz["transforms"][1]["dy"] = ball_pos[1] / BALL_RADIUS;
    rotation_viz["transforms"][1]["dz"] = ball_pos[2] / BALL_RADIUS;

    Eigen::Vector3d aor = _state.ball_pose.axis_of_rotation * 2.4;
    rotation_viz["p1"][0] = 0.;
    rotation_viz["p1"][1] = 0.;
    rotation_viz["p1"][2] = 0.;
    rotation_viz["p2"][0] = aor[0];
    rotation_viz["p2"][1] = aor[1];
    rotation_viz["p2"][2] = aor[2];
    rotation_viz["r"] = 0.25;

    objects.push_back(std::unique_ptr<raytracer::Cylinder>(
        new raytracer::Cylinder{rotation_viz, rotation_viz_material}));
  }

  // Next make the table
  YAML::Node table_node;
  const Eigen::Vector3d table_pos = get_table_pos();
  table_node["primative"] = "cylinder";
  table_node["transforms"][0]["type"] = "translate";
  table_node["transforms"][0]["dx"] = table_pos[0];
  table_node["transforms"][0]["dy"] = table_pos[1];
  table_node["transforms"][0]["dz"] = table_pos[2];

  const Eigen::Vector3d normal = get_table_normal_vec() * TABLE_HEIGHT;
  // TODO: switch pivoting on the bottom
  table_node["p1"][0] = -normal[0];
  table_node["p1"][1] = -normal[1];
  table_node["p1"][2] = -normal[2];
  table_node["p2"][0] = 0.;
  table_node["p2"][1] = 0.;
  table_node["p2"][2] = 0.;
  table_node["r"] = TABLE_RADIUS;

  objects.push_back(std::unique_ptr<raytracer::Cylinder>(
      new raytracer::Cylinder{table_node, table_material}));

  return raytracer::ObjectVector(std::move(objects));
};

} // namespace control_final
