#pragma once
#include "control_final/controller/reference.h"
#include "control_final/model/state.h"

namespace control_final {

class Controller {
public:
  Controller() {};
  virtual void react(std::vector<char> &pixs, Reference &u) = 0;

protected:
  State _predict_state(std::vector<char> &pixs);
};

} // namespace control_final
