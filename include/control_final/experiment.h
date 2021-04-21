#pragma once

#include "yaml-cpp/yaml.h"
#include <map>
#include <string>
#include <vector>

namespace control_final {
class Experiment {
public:
  Experiment(const YAML::Node &node);
  void run();

private:
  YAML::Node m_base_config;
  std::vector<std::map<std::string, YAML::Node>> m_all_configs;
};
} // namespace control_final
