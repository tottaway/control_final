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
    : _xres(xres), _yres(yres), _camera_pos(camera_pos),
      _camera_dir(camera_dir) {
  _renderer = std::make_unique<raytracer::Renderer>(_xres, _yres, _camera_pos,
                                                    _camera_dir);
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

  _xres = file["xres"].as<unsigned>();
  _yres = file["yres"].as<unsigned>();
  _camera_pos << file["camera_pos"][0].as<double>(),
      file["camera_pos"][1].as<double>(), file["camera_pos"][2].as<double>();
  _camera_dir << file["camera_dir"][0].as<double>(),
      file["camera_dir"][1].as<double>(), file["camera_dir"][2].as<double>();

  _renderer = std::make_unique<raytracer::Renderer>(_xres, _yres, _camera_pos,
                                                    _camera_dir);
}

void Sensor::observe(const Environment &env, std::vector<char> &pixs) {
  auto ov = env.to_object_vector();

  // filename is empty since we don't want to write to disk
  std::string filename = "";
  _renderer->render(filename, pixs, ov, RenderingConfigs::lights);
}

} // namespace control_final
