#include "control_final/sensor/sensor.h"
#include "raytracer/data_structures/object_vector.h"
#include "raytracer/util.h"
#include "raytracer/light.h"

#include <Eigen/Dense>
#include <vector>

// Here these colors need to be BGR
static const raytracer::Color white{1, 1, 1};
static const raytracer::Color red{0.2, 0.2, 1};
static const raytracer::Color blue{1, 0.2, 0.2};
static const raytracer::Color green{0.2, 1, 0.2};

static const raytracer::Light light1 {
  .I = 2, .type = raytracer::LightType::POINT_LIGHT, .color = white,
  .pose {
    4, -1, 6
  }
};

static const raytracer::Light light2 {
  .I = 2, .type = raytracer::LightType::DIRECTIONAL_LIGHT, .color = white,
  .dir {
    1, 0, -0.5
  }
};

static const std::vector<raytracer::Light> lights{light1, light2};

namespace control_final {

Sensor::Sensor(const unsigned x_res, const unsigned y_res,
               const Eigen::Vector3d camera,
               const Eigen::Vector3d camera_direction)
    : _x_res(x_res), _y_res(y_res),
      _renderer(x_res, y_res, camera, camera_direction) {}

void Sensor::render(const State &true_state, std::vector<char> &pixs) {

  auto ov = true_state.to_object_vector();

  // filename is empty since we don't want to write to disk
  std::string filename = "";
  _renderer.render(filename, pixs, ov, lights);
}

} // namespace control_final
