#include "control_final/model/state.h"

#include "raytracer/data_structures/object_vector.h"
#include "raytracer/light.h"
#include "raytracer/material.h"
#include "raytracer/shapes.h"
#include "raytracer/util.h"

#include "yaml.h"

#include <memory>
#include <vector>

namespace control_final {

static const raytracer::Color ball_color{0, 165. / 255, 1};
static const raytracer::Color white{1, 1, 1};

static const raytracer::Material ball_material{.kd = 0.75,
                                               .ks = 0.20,
                                               .specular_exponent = 0.5,
                                               .diffuse_color = ball_color,
                                               .specular_color = white};

static const raytracer::Material table_material{.kd = 0.5,
                                                .ks = 0.24,
                                                .specular_exponent = 0.5,
                                                .diffuse_color = white,
                                                .specular_color = white};

raytracer::ObjectVector State::to_object_vector() const {
  // First make the table
  YAML::Node ball_node;
  ball_node["primative"] = "sphere";
  ball_node["transforms"][0]["type"] = "translate";
  ball_node["transforms"][0]["dx"] = ball_pose.x;
  ball_node["transforms"][0]["dy"] = ball_pose.y;
  ball_node["transforms"][0]["dz"] = ball_pose.z;

  std::vector<std::unique_ptr<raytracer::SceneObject>> objects;

  // TODO: for some reason make_unique wasn't working so I'm doing this for
  // now
  objects.push_back(std::unique_ptr<raytracer::Sphere>(
      new raytracer::Sphere{ball_node, ball_material}));

  // TODO: render table as well

  
  return raytracer::ObjectVector(std::move(objects));
};

} // namespace control_final
