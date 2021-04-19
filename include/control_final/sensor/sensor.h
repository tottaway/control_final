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

  size_t get_xres() const { return _xres; }
  size_t get_yres() const { return _yres; }
  Eigen::Vector3d get_camera_pos() const { return _camera_pos; }
  Eigen::Vector3d get_camera_dir() const { return _camera_dir; }
  size_t get_pixs_size() const { return _xres * _yres * 3; }

private:
  unsigned _xres;
  unsigned _yres;
  Eigen::Vector3d _camera_pos;
  Eigen::Vector3d _camera_dir;

  std::unique_ptr<raytracer::Renderer> _renderer;
  void process();
};

} // namespace control_final
