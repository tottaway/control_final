#pragma once
#include "control_final/controller/reference.h"
#include "control_final/sensor/sensor.h"

namespace control_final {

class Controller {
public:
  Controller(const std::string &filename){};
  // we take in the sensor so that we can have access to parameters like the
  // camera location and dir
  virtual void react(std::vector<char> &pixs, Reference &u,
                     const Sensor &sensor) = 0;

protected:
  State predict_state(std::vector<char> &pixs, const Sensor &sensor);
};

} // namespace control_final
