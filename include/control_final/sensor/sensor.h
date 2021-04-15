#pragma once
#include "control_final/model/state.h"

#include "raytracer/data_structures/object_vector.h"
#include "raytracer/light.h"
#include "raytracer/renderer.h"
#include <Magick++.h>

#include <Eigen/Dense>
#include <vector>

namespace control_final {

class Sensor {
public:
  Sensor(const unsigned x_res, const unsigned y_res,
         const Eigen::Vector3d camera, const Eigen::Vector3d camera_direction);

  void measure(const State &true_state, State &measured_state);
  void render(const State &true_state, std::vector<char> &pixs,
              const std::vector<raytracer::Light> &lights);

private:
  const unsigned _x_res;
  const unsigned _y_res;

  raytracer::Renderer _renderer;
  void process();
};

} // namespace control_final
