#pragma once

#include "control_final/model/environment.h"
#include "control_final/sensor/sensor.h"
#include "control_final/controller/reference.h"

#include <vector>
#include <memory>

namespace control_final {

template<typename T>
concept ControllerType = requires(T c,
        const State measured_state, Reference u) {
    // must support reacting to state
    c.react(measured_state, u);
};

template <typename ControllerType>
class Simulator {
    public:
        Simulator();
        void step();

        void get_state(State &state);

    private:
        Sensor _sensor;
        Environment _env;
        ControllerType _controller;
};

}
