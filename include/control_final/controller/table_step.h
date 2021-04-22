
#pragma once
#include "control_final/controller/controller.h"
#include "control_final/sensor/sensor.h"

#include "yaml-cpp/yaml.h"

namespace control_final {

class TableStepController : public Controller {
public:
  TableStepController(const YAML::Node &node);
  void react(std::vector<char> &pixs, const Sensor &sensor,
             const double t) override;
};
} // namespace control_final
