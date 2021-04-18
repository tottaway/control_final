#pragma once
#include "control_final/model/environment.h"
#include "control_final/model/state.h"

#include "raytracer/renderer.h"
#include <Magick++.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

namespace control_final {

class Sensor {
public:
  Sensor(const unsigned x_res, const unsigned y_res,
         const Eigen::Vector3d camera_pos, const Eigen::Vector3d camera_dir);

  Sensor(const std::string &filename);

  void observe(const Environment &env, std::vector<char> &pixs);

  size_t get_xres() { return _xres; }
  size_t get_yres() { return _yres; }
  size_t get_pixs_size() { return _xres * _yres * 3; }

private:
  unsigned _xres;
  unsigned _yres;
  Eigen::Vector3d _camera_pos;
  Eigen::Vector3d _camera_dir;

  std::unique_ptr<raytracer::Renderer> _renderer;
  void process();
};

} // namespace control_final
