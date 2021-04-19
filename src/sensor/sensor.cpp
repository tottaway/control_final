#include "control_final/sensor/sensor.h"
#include "control_final/model/environment.h"
#include "control_final/model/render_config.h"

#include "raytracer/renderer.h"

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <vector>

// Here these colors need to be BGR

namespace control_final {

Sensor::Sensor(const unsigned xres, const unsigned yres,
               const Eigen::Vector3d camera_pos,
               const Eigen::Vector3d camera_dir)
    : m_xres(xres), m_yres(yres), m_camera_pos(camera_pos),
      m_camera_dir(camera_dir) {
  m_renderer = std::make_unique<raytracer::Renderer>(
      m_xres, m_yres, m_camera_pos, m_camera_dir);
}

Sensor::Sensor(const std::string &filename) {
  auto file = YAML::LoadFile(filename);

  // error check
  if (!file["xres"]) {
    std::cout << "Did not specify xres in " << filename << std::endl;
    exit(1);
  } else if (!file["yres"]) {
    std::cout << "Did not specify yres in " << filename << std::endl;
    exit(1);
  } else if (!file["camera_pos"]) {
    std::cout << "Did not specify camera_pos in " << filename << std::endl;
    exit(1);
  } else if (!file["camera_dir"]) {
    std::cout << "Did not specify camera_dir in " << filename << std::endl;
    exit(1);
  }

  m_xres = file["xres"].as<unsigned>();
  m_yres = file["yres"].as<unsigned>();
  m_camera_pos << file["camera_pos"][0].as<double>(),
      file["camera_pos"][1].as<double>(), file["camera_pos"][2].as<double>();
  m_camera_dir << file["camera_dir"][0].as<double>(),
      file["camera_dir"][1].as<double>(), file["camera_dir"][2].as<double>();

  m_renderer = std::make_unique<raytracer::Renderer>(
      m_xres, m_yres, m_camera_pos, m_camera_dir);
}

void Sensor::observe(const Environment &env, std::vector<char> &pixs) {
  auto ov = env.to_object_vector();

  // filename is empty since we don't want to write to disk
  std::string filename = "";
  m_renderer->render(filename, pixs, ov, RenderingConfigs::lights);
}

} // namespace control_final
