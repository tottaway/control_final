#pragma once
#include "raytracer/data_structures/object_vector.h"
#include "raytracer/light.h"
#include "raytracer/material.h"
#include "raytracer/shapes.h"
#include "raytracer/util.h"

namespace control_final {
// These are read in from config/render_config.yaml
struct RenderingConfigs {
  RenderingConfigs(const YAML::Node &node);

  raytracer::Material ball_material;

  raytracer::Material table_material;

  raytracer::Material rotation_viz_material;
  bool show_rotation_viz;

  std::vector<raytracer::Light> lights;
};

} // namespace control_final
