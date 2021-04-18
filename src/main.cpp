#include "control_final/simulator.h"

#include <Eigen/Dense>
#include <iostream>

using namespace control_final;
int main(int argc, char *argv[]) {
  Simulator sim("../config/");
  sim.run();
}
