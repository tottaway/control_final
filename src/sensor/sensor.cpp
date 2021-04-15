#include "control_final/sensor/sensor.h"
#include "raytracer/data_structures/object_vector.h"
#include "raytracer/light.h"

#include <Eigen/Dense>
#include <vector>

namespace control_final {

Sensor::Sensor(const unsigned x_res, const unsigned y_res,
               const Eigen::Vector3d camera,
               const Eigen::Vector3d camera_direction)
    : _x_res(x_res), _y_res(y_res),
      _renderer(x_res, y_res, camera, camera_direction) {}

void Sensor::render(const State &true_state, std::vector<char> &pixs,
                    const std::vector<raytracer::Light> &lights) {

  auto ov = true_state.to_object_vector();

  // filename is empty since we don't want to write to disk
  std::string filename = "";
  _renderer.render(filename, pixs, ov, lights);
}

} // namespace control_final