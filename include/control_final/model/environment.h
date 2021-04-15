#pragma once
#include "control_final/model/state.h"
#include "control_final/controller/reference.h"

namespace control_final{

class Environment {
    public:
        Environment();
        void step(const Reference &u);

        State get_state() {return _state;};

    private:
        State _state;
};

}
