#include "control_final/controller/table_step.h"

namespace control_final {
TableStepController::TableStepController(const YAML::Node &node) : Controller(node) {
  auto controller_node = node["controller"];
  m_ref_theta_x = 0;
  m_ref_theta_y = 0;
}

void TableStepController::react(std::vector<char> &pixs, const Sensor &sensor,
                          const double t) {
  m_ref_theta_x = 0.2;
  m_ref_theta_y = 0.2;
}
} // namespace control_final
