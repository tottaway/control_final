#include "control_final/sensor/sensor.h"
#include "control_final/model/environment.h"
#include "control_final/model/render_config.h"

#include "raytracer/renderer.h"

#include "yaml.h"
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// Here these colors need to be BGR

namespace control_final {

Sensor::Sensor(const YAML::Node &node, const std::string &sensor_name)
    : m_render_configs(node) {
  auto sensor_node = node[sensor_name];

  // error check
  if (!sensor_node["xres"]) {
    std::cout << "Did not specify xres in sensor: " << sensor_name << std::endl;
    exit(1);
  } else if (!sensor_node["yres"]) {
    std::cout << "Did not specify yres in sensor: " << sensor_name << std::endl;
    exit(1);
  } else if (!sensor_node["camera_pos"]) {
    std::cout << "Did not specify camera_pos in sensor: " << sensor_name << std::endl;
    exit(1);
  } else if (!sensor_node["camera_dir"]) {
    std::cout << "Did not specify camera_dir in sensor: " << sensor_name << std::endl;
    exit(1);
  }

  m_xres = sensor_node["xres"].as<unsigned>();
  m_yres = sensor_node["yres"].as<unsigned>();
  m_camera_pos << sensor_node["camera_pos"][0].as<double>(),
      sensor_node["camera_pos"][1].as<double>(),
      sensor_node["camera_pos"][2].as<double>();
  m_camera_dir << sensor_node["camera_dir"][0].as<double>(),
      sensor_node["camera_dir"][1].as<double>(),
      sensor_node["camera_dir"][2].as<double>();

  m_renderer = std::make_unique<raytracer::Renderer>(
      m_xres, m_yres, m_camera_pos, m_camera_dir);
}

void Sensor::observe(const Environment &env, std::vector<char> &pixs) {
  make_object_vector(env);

  // filename is empty since we don't want to write to disk
  std::string filename = "";
  if (m_ov.has_value()) {
    m_renderer->render(filename, pixs, m_ov.value(), m_render_configs.lights);
  } else {
    std::cout << "m_ov was null in Sensor::observe" << std::endl;
    exit(1);
  }
}

// and have sensor take in env and not state
void Sensor::make_object_vector(const Environment &env) {
  std::vector<std::unique_ptr<raytracer::SceneObject>> objects;

  // First make the ball
  YAML::Node ball_node;

  const Eigen::Vector3d ball_pos = env.get_ball_pos();
  ball_node["primative"] = "sphere";
  ball_node["transforms"][0]["type"] = "scale";
  ball_node["transforms"][0]["scale_factor"] = env.ball_radius;

  // Note that since we've scaled everything we need to also scale the
  // translations
  ball_node["transforms"][1]["type"] = "translate";
  ball_node["transforms"][1]["dx"] = ball_pos[0] / env.ball_radius;
  ball_node["transforms"][1]["dy"] = ball_pos[1] / env.ball_radius;
  ball_node["transforms"][1]["dz"] = ball_pos[2] / env.ball_radius;

  // TODO: for some reason make_unique wasn't working so I'm doing this for
  // now
  objects.push_back(std::unique_ptr<raytracer::Sphere>(
      new raytracer::Sphere{ball_node, m_render_configs.ball_material}));

  // Extra cylinder to visualize rotation
  if (m_render_configs.show_rotation_viz) {
    YAML::Node rotation_viz;
    rotation_viz["primative"] = "cylinder";
    rotation_viz["transforms"][0]["type"] = "scale";
    rotation_viz["transforms"][0]["scale_factor"] = env.ball_radius;

    rotation_viz["transforms"][1]["type"] = "translate";
    rotation_viz["transforms"][1]["dx"] = ball_pos[0] / env.ball_radius;
    rotation_viz["transforms"][1]["dy"] = ball_pos[1] / env.ball_radius;
    rotation_viz["transforms"][1]["dz"] = ball_pos[2] / env.ball_radius;

    Eigen::Vector3d aor = env.get_ball_aor() * 2.4;
    rotation_viz["p1"][0] = 0.;
    rotation_viz["p1"][1] = 0.;
    rotation_viz["p1"][2] = 0.;
    rotation_viz["p2"][0] = aor[0];
    rotation_viz["p2"][1] = aor[1];
    rotation_viz["p2"][2] = aor[2];
    rotation_viz["r"] = 0.25;

    objects.push_back(
        std::unique_ptr<raytracer::Cylinder>(new raytracer::Cylinder{
            rotation_viz, m_render_configs.rotation_viz_material}));
  }

  // Next make the table
  YAML::Node table_node;
  const Eigen::Vector3d table_pos = env.get_table_pos();
  table_node["primative"] = "cylinder";
  table_node["transforms"][0]["type"] = "translate";
  table_node["transforms"][0]["dx"] = table_pos[0];
  table_node["transforms"][0]["dy"] = table_pos[1];
  table_node["transforms"][0]["dz"] = table_pos[2];

  const Eigen::Vector3d normal = env.get_table_normal_vec() * env.table_height;
  // TODO: switch pivoting on the bottom
  table_node["p1"][0] = -normal[0];
  table_node["p1"][1] = -normal[1];
  table_node["p1"][2] = -normal[2];
  table_node["p2"][0] = 0.;
  table_node["p2"][1] = 0.;
  table_node["p2"][2] = 0.;
  table_node["r"] = env.table_radius;

  objects.push_back(std::unique_ptr<raytracer::Cylinder>(
      new raytracer::Cylinder{table_node, m_render_configs.table_material}));

  m_ov = raytracer::ObjectVector(std::move(objects));
};

} // namespace control_final
