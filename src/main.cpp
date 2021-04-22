#include "control_final/simulator.h"
#include "control_final/experiment.h"

#include "yaml-cpp/yaml.h"

using namespace control_final;

int main(int argc, char *argv[]) {
  // TODO: get file name from arguments
  /* auto experiment_config = YAML::LoadFile("../experiments/pid_grid_search.yaml"); */
  /* Experiment experiment(experiment_config); */
  /* experiment.run(); */

  auto config = YAML::LoadFile("../config/pid.yaml");
  Simulator sim(config);
  sim.run();
}
