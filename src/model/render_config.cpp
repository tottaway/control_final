#include "control_final/model/render_config.h"

#include "raytracer/light.h"
#include "raytracer/material.h"
#include "raytracer/util.h"

#include "yaml-cpp/yaml.h"
#include <iostream>
#include <map>
#include <memory>
#include <string>

namespace control_final {
RenderingConfigs::RenderingConfigs(const YAML::Node &node) {
  auto render_node = node["render"];

  // This part is ripped straight from the raytracer project
  // error checking
  if (!render_node["materials"]) {
    std::cout << "Did not specify materials in render" << std::endl;
    exit(1);
  }

  auto materials_node = render_node["materials"];
  std::map<std::string, raytracer::Material> materials;
  for (std::size_t i = 0; i < materials_node.size(); i++) {
    // Note colors are specified in the range [0, 255], but I need then in
    // [0, 1]
    raytracer::Color diffuse_color{
        materials_node[i]["diffuse_color"][0].as<float>(),
        materials_node[i]["diffuse_color"][1].as<float>(),
        materials_node[i]["diffuse_color"][2].as<float>()};
    diffuse_color /= 255;
    raytracer::Color specular_color{
        materials_node[i]["specular_color"][0].as<float>(),
        materials_node[i]["specular_color"][1].as<float>(),
        materials_node[i]["specular_color"][2].as<float>()};
    specular_color /= 255;
    raytracer::Material m{
        .kd = materials_node[i]["kd"].as<float>(),
        .ks = materials_node[i]["ks"].as<float>(),
        .specular_exponent = materials_node[i]["specular_exponent"].as<float>(),
        .diffuse_color = diffuse_color,
        .specular_color = specular_color};
    materials[materials_node[i]["name"].as<std::string>()] = m;
  }

  bool show_rotation_viz = false;
  if (render_node["show_rotation_viz"]) {
    YAML::convert<bool>::decode(render_node["show_rotation_viz"],
                                show_rotation_viz);
  }

  // error checking
  if (!materials.count("ball_material")) {
    std::cout << "Did not specify ball_material in render" << std::endl;
    exit(1);
  } else if (!materials.count("table_material")) {
    std::cout << "Did not specify table_material in render" << std::endl;
    exit(1);
  } else if (render_node["show_rotation_viz"] && show_rotation_viz &&
             !materials.count("rotation_viz_material")) {
    std::cout << "show_rotation_viz was set to true, but no "
                 "rotation_viz_material was definied in render"
              << std::endl;
    exit(1);
  }

  ball_material = materials["ball_material"];
  table_material = materials["table_material"];

  if (show_rotation_viz) {
    rotation_viz_material = materials["rotation_viz_material"];
  }

  // error checking
  if (!render_node["lights"]) {
    std::cout << "Did not specify lights in render" << std::endl;
    exit(1);
  }

  // parse the lights (this was also ripped from the raytracer project
  auto lights_node = render_node["lights"];
  if (lights_node.size() == 0) {
    std::cout << "Lights node found in render"
              << " but no lights were defined" << std::endl;
    exit(1);
  }
  for (std::size_t i = 0; i < lights_node.size(); i++) {
    raytracer::Color color{lights_node[i]["color"][0].as<float>(),
                           lights_node[i]["color"][1].as<float>(),
                           lights_node[i]["color"][2].as<float>()};
    color /= 255;
    raytracer::Light l = {
        .I = lights_node[i]["intensity"].as<float>(),
        .color = color,
    };

    if (lights_node[i]["type"].as<std::string>() == "point") {
      Eigen::Vector3d pose{lights_node[i]["pose"][0].as<double>(),
                           lights_node[i]["pose"][1].as<double>(),
                           lights_node[i]["pose"][2].as<double>()};
      l.type = raytracer::LightType::POINT_LIGHT;
      l.pose = pose;
    } else if (lights_node[i]["type"].as<std::string>() == "directional") {
      Eigen::Vector3d dir{lights_node[i]["dir"][0].as<double>(),
                          lights_node[i]["dir"][1].as<double>(),
                          lights_node[i]["dir"][2].as<double>()};
      l.type = raytracer::LightType::DIRECTIONAL_LIGHT;
      l.dir = dir;
    }

    lights.push_back(l);
  }
}
} // namespace control_final
