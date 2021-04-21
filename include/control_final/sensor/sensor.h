#pragma once
#include "control_final/model/environment.h"
#include "control_final/model/render_config.h"
#include "control_final/model/state.h"

#include "raytracer/data_structures/object_vector.h"
#include "raytracer/renderer.h"

#include "yaml-cpp/yaml.h"
#include <Eigen/Dense>
#include <Magick++.h>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace control_final {

class Sensor {
public:
  Sensor(const YAML::Node &node, const std::string &sensor_name);

  void observe(const Environment &env, std::vector<char> &pixs);

  unsigned get_xres() const { return m_xres; }
  unsigned get_yres() const { return m_yres; }
  Eigen::Vector3d get_camera_pos() const { return m_camera_pos; }
  Eigen::Vector3d get_camera_dir() const { return m_camera_dir; }
  unsigned get_pixs_size() const { return m_xres * m_yres * 3; }

private:
  unsigned m_xres;
  unsigned m_yres;
  Eigen::Vector3d m_camera_pos;
  Eigen::Vector3d m_camera_dir;

  std::unique_ptr<raytracer::Renderer> m_renderer;
  void process();

  // Makes object vector to pass to raytracer
  void make_object_vector(const Environment &env);
  // This is an optional since it doesn't make sense to initialize it
  // immediately
  std::optional<raytracer::ObjectVector> m_ov;

  RenderingConfigs m_render_configs;
};

} // namespace control_final
