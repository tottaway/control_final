#pragma once
#include "raytracer/data_structures/object_vector.h"
#include "raytracer/light.h"
#include "raytracer/material.h"
#include "raytracer/shapes.h"
#include "raytracer/util.h"

namespace control_final {
// These are read in from config/render_config.yaml
namespace RenderingConfigs {
extern raytracer::Material ball_material;

extern raytracer::Material table_material;

extern raytracer::Material rotation_viz_material;
extern bool show_rotation_viz;

extern std::vector<raytracer::Light> lights;
}; // namespace RenderingConfigs

void parse_render_configs(const std::string &filename);

} // namespace control_final
