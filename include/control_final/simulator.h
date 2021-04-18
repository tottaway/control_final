#pragma once

#include "control_final/controller/controller.h"
#include "control_final/controller/reference.h"
#include "control_final/model/environment.h"
#include "control_final/sensor/sensor.h"

#include "movie.h"
#include <memory>
#include <vector>

namespace control_final {

class Simulator {
public:
  Simulator(const std::string &config_dir);
  void run();

  State get_state() { return _env.get_state(); };

private:
  Sensor _sensor;
  Environment _env;
  std::unique_ptr<Controller> _controller;

  // Total time to run the simulation
  double _T;
  unsigned _fps;

  // This is the camera which the observer is watching through
  bool _make_observer_video;
  // This are pointers so that they can be null if the flag above is false
  std::unique_ptr<Sensor> _observer;
  std::unique_ptr<MovieWriter> _writer;

  void _parse_sim_configs(const std::string &sim_file_name);
};

} // namespace control_final
