#pragma once
#include "control_final/controller/reference.h"

namespace control_final {

class PIDController {
    public:
        PIDController();
    void react(const State &measured_state, Reference &u);
};
}
