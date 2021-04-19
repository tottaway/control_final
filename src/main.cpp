#include "control_final/simulator.h"

#include "yaml.h"

using namespace control_final;

int main(int argc, char *argv[]) {
  auto config = YAML::LoadFile("../config/pid.yaml");
  Simulator sim(config);
  sim.run();
}
