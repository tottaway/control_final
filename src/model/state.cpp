#include "control_final/model/state.h"
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

namespace control_final {

// Colors are BGR
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
                                                .diffuse_color = table_color,
                                                .specular_color = white};

/* static const raytracer::Material rotation_viz_material{.kd = 0.75, */
/*                                                        .ks = 0.05, */
/*                                                        .specular_exponent = 4, */
/*                                                        .diffuse_color = red, */
/*                                                        .specular_color = white}; */

// TODO: move this function to Environment class
// and have sensor take in env and not state
raytracer::ObjectVector State::to_object_vector() const {
  std::vector<std::unique_ptr<raytracer::SceneObject>> objects;

  // First make the ball
  YAML::Node ball_node;

  ball_node["primative"] = "sphere";
  ball_node["transforms"][0]["type"] = "scale";
  ball_node["transforms"][0]["scale_factor"] = Environment::BALL_RADIUS;

  ball_node["transforms"][1]["type"] = "translate";
  ball_node["transforms"][1]["dx"] = ball_pose.x / Environment::BALL_RADIUS;
  ball_node["transforms"][1]["dy"] = ball_pose.y / Environment::BALL_RADIUS;
  ball_node["transforms"][1]["dz"] = ball_pose.z / Environment::BALL_RADIUS;

  // TODO: for some reason make_unique wasn't working so I'm doing this for
  // now
  objects.push_back(std::unique_ptr<raytracer::Sphere>(
      new raytracer::Sphere{ball_node, ball_material}));

  // Extra cylinder to visualize rotation
  /* YAML::Node rotation_viz; */
  /* rotation_viz["primative"] = "cylinder"; */
  /* rotation_viz["transforms"][0]["type"] = "scale"; */
  /* rotation_viz["transforms"][0]["scale_factor"] = Environment::BALL_RADIUS; */

  /* rotation_viz["transforms"][1]["type"] = "translate"; */
  /* rotation_viz["transforms"][1]["dx"] = ball_pose.x / Environment::BALL_RADIUS; */
  /* rotation_viz["transforms"][1]["dy"] = ball_pose.y / Environment::BALL_RADIUS; */
  /* rotation_viz["transforms"][1]["dz"] = ball_pose.z / Environment::BALL_RADIUS; */

  /* // TODO: change this to use get_table_normal_vec */
  /* Eigen::Vector3d aor = ball_pose.axis_of_rotation * 2.4; */
  /* rotation_viz["p1"][0] = 0.; */
  /* rotation_viz["p1"][1] = 0.; */
  /* rotation_viz["p1"][2] = 0.; */
  /* rotation_viz["p2"][0] = aor[0]; */
  /* rotation_viz["p2"][1] = aor[1]; */
  /* rotation_viz["p2"][2] = aor[2]; */
  /* rotation_viz["r"] = 0.25; */

  /* objects.push_back(std::unique_ptr<raytracer::Cylinder>( */
  /*     new raytracer::Cylinder{rotation_viz, rotation_viz_material})); */

  // Next make the table
  YAML::Node table_node;
  table_node["primative"] = "cylinder";
  table_node["transforms"][0]["type"] = "translate";
  table_node["transforms"][0]["dx"] = table_pose.x;
  table_node["transforms"][0]["dy"] = table_pose.y;
  table_node["transforms"][0]["dz"] = table_pose.z;

  double height = Environment::TABLE_HEIGHT;
  // TODO: change this to use get_table_normal_vec
  table_node["p1"][0] = -height * sin(table_pose.theta_y);
  table_node["p1"][1] = -height * sin(table_pose.theta_x);
  table_node["p1"][2] =
      -sqrt(pow(height, 2) - (pow(height * sin(table_pose.theta_x), 2) +
                              pow(height * sin(table_pose.theta_y), 2)));
  table_node["p2"][0] = 0.;
  table_node["p2"][1] = 0.;
  table_node["p2"][2] = 0.;
  table_node["r"] = Environment::TABLE_RADIUS;

  objects.push_back(std::unique_ptr<raytracer::Cylinder>(
      new raytracer::Cylinder{table_node, table_material}));

  // TODO: render table as well

  return raytracer::ObjectVector(std::move(objects));
};

} // namespace control_final
