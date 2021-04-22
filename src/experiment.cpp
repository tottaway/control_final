#include "control_final/experiment.h"
#include "control_final/simulator.h"

#include "yaml-cpp/yaml.h"
#include <iostream>
#include <map>
#include <numeric>
#include <string>
#include <vector>

namespace control_final {
// modified from https://stackoverflow.com/a/31169617
// This is kind of gross
template <typename TKey, typename TValue>
static void cartesian(const std::map<TKey, std::vector<TValue>> &m,
                      std::vector<std::map<TKey, TValue>> &o) {
  auto product = [](const long long a,
                    const std::pair<TKey, std::vector<TValue>> &b) {
    return a * b.second.size();
  };
  const int N = accumulate(m.begin(), m.end(), 1LL, product);
  for (long long n = 0; n < N; ++n) {
    lldiv_t q{n, 0};
    std::map<TKey, TValue> curr_map;
    for (const auto &item : m) {
      q = div(q.quot, item.second.size());
      curr_map[item.first] = item.second[q.rem];
    }
    o.push_back(curr_map);
  }
}

// source https://stackoverflow.com/a/49602621
template <typename T, typename Iter>
static void set_config_param(YAML::Node node, Iter begin, Iter end, T value) {
  if (begin == end) {
    return;
  }
  const auto &tag = *begin;
  if (std::next(begin) == end) {
    node[tag] = value;
    return;
  }
  if (!node[tag]) {
    node[tag] = YAML::Node(YAML::NodeType::Map);
  }
  set_config_param(node[tag], std::next(begin), end, value);
}

Experiment::Experiment(const YAML::Node &node) {
  const auto parameters =
      node["experiment"]["parameters"].as<std::vector<std::string>>();

  const auto nparamaters = parameters.size();
  // Map from parameter name to vector of values
  std::map<std::string, std::vector<YAML::Node>> all_param_values;
  for (size_t i = 0; i < nparamaters; i++) {
    // name of field in experiment which contains param values
    const std::string param_name = parameters[i];
    std::string field_name = param_name;

    const auto field_name_len = field_name.size();
    for (size_t j = 0; j < field_name_len; j++) {
      if (field_name[j] == ' ')
        field_name[j] = '_';
    }

    const YAML::Node field_node = node["experiment"][field_name];
    const auto nvalues = field_node.size();
    std::vector<YAML::Node> values;
    for (size_t j = 0; j < nvalues; j++) {
      values.push_back(field_node[j]);
    }
    all_param_values[param_name] = values;
  }

  cartesian(all_param_values, m_all_configs);

  const auto nconfigs = m_all_configs.size();
  for (size_t i = 0; i < nconfigs; i++) {
    std::string log_base_name = "";
    const auto log_dir = node["experiment"]["log_dir"].as<std::string>();
    for (const auto &item : m_all_configs[i]) {
      auto last_space_idx = item.first.find_last_of(' ');
      log_base_name += "_" + item.first.substr(last_space_idx + 1) + "_" +
                       item.second.as<std::string>();
    }
    log_base_name += ".csv";

    m_all_configs[i]["log_sensor"] =
        YAML::Load(log_dir + "sensor" + log_base_name);
    m_all_configs[i]["log_controller"] =
        YAML::Load(log_dir + "controller" + log_base_name);
    m_all_configs[i]["log_env"] = YAML::Load(log_dir + "env" + log_base_name);
  }

  m_base_config = node["base_config"];
}

void Experiment::run() {
  const auto nconfigs = m_all_configs.size();
  for (size_t i = 0; i < nconfigs; i++) {

    // edit base config
    for (const auto &item : m_all_configs[i]) {
      // split the param name on spaces
      size_t last = 0;
      size_t next = 0;
      const auto param_name = item.first;
      std::vector<std::string> param_vector;
      while ((next = param_name.find(' ', last)) != std::string::npos) {
        param_vector.push_back(param_name.substr(last, next - last));
        last = next + 1;
      }
      param_vector.push_back(param_name.substr(last));

      // recusively apply indices
      set_config_param(m_base_config, param_vector.begin(), param_vector.end(),
                       item.second);
    }

    std::cout << "Starting sim number " << i+1 << " of " << nconfigs << std::endl;
    // run simulation
    Simulator sim(m_base_config);
    sim.run();
  }
}

} // namespace control_final
