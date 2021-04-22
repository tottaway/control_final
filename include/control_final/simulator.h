#pragma once

#include "control_final/controller/controller.h"
#include "control_final/model/environment.h"
#include "control_final/sensor/sensor.h"

#include "yaml-cpp/yaml.h"
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>
#include <fstream>

namespace control_final {

class Simulator {
public:
  Simulator(const YAML::Node &node);
  void run();

private:
  Sensor m_sensor;
  Environment m_env;
  std::unique_ptr<Controller> m_controller;

  // Total time to run the simulation
  double m_T;
  unsigned m_fps;

  // This is the camera which the observer is watching through
  bool m_make_observer_video;
  // This are pointers so that they can be null if the flag above is false
  std::unique_ptr<Sensor> m_observer;
  std::unique_ptr<cv::VideoWriter> m_observer_writer;

  // This is the camera which the observer is watching through
  bool m_make_sensor_video;
  std::unique_ptr<cv::VideoWriter> m_sensor_writer;

  void parse_sim_configs(const YAML::Node &node);

  bool m_env_logging_enabled = false;
  std::string m_env_log_filename;
  std::ofstream m_env_log;

  bool m_sensor_logging_enabled = false;
  std::string m_sensor_log_filename;
  std::ofstream m_sensor_log;

  bool m_controller_logging_enabled = false;
  std::string m_controller_log_filename;
  std::ofstream m_controller_log;

  void init_logs();
  void close_logs();
  void write_env_log(const double t);
  void write_sensor_log(const double t);
  void write_controller_log(const double t, const ControllerOutput &u);
};

} // namespace control_final


