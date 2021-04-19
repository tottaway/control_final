#include "control_final/simulator.h"
#include "control_final/controller/controller.h"
#include "control_final/controller/pid_controller.h"
#include "control_final/model/environment.h"
#include "control_final/model/render_config.h"
#include "control_final/model/state.h"
#include "control_final/sensor/sensor.h"
#include "control_final/util/parse_configs.h"

#include "movie.h"
#include "yaml.h"
#include <Eigen/Dense>
#include <iostream>
#include <map>
#include <memory>
#include <string>

namespace control_final {

void Simulator::_parse_sim_configs(const std::string &config_dir) {
  auto sim_file_name = config_dir + "/sim.yaml";
  auto file = YAML::LoadFile(sim_file_name);

  // get information related to observer
  if (file["make_observer_video"] && file["make_observer_video"].as<bool>()) {
    _make_observer_video = true;
    const auto observer_config_file = config_dir + "/observer.yaml";

    // This seems pretty gross but I want to make it optional to provide config
    // file for the observer so I need to check the sim config file before
    // constructing the observer which means I can't do this in the initializer
    // list. There is probably a better way of doing this
    _observer = std::make_unique<Sensor>(observer_config_file);

    // have sensible default for output file
    if (!file["observer_video_filename"]) {
      _observer_writer = std::make_unique<MovieWriter>(
          "observer_out", _observer->get_xres(), _observer->get_yres());
    } else {
      const auto out_filename =
          file["observer_video_filename"].as<std::string>();
      _observer_writer = std::make_unique<MovieWriter>(
          out_filename, _observer->get_xres(), _observer->get_yres());
    }
  } else {
    _make_observer_video = false;
  }

  // give support for saving sensor video
  if (file["make_sensor_video"] && file["make_sensor_video"].as<bool>()) {
    _make_sensor_video = true;
    const auto sensor_config_file = config_dir + "/sensor.yaml";

    // have sensible default for output file
    if (!file["sensor_video_filename"]) {
      _sensor_writer = std::make_unique<MovieWriter>(
          "sensor_out", _sensor.get_xres(), _sensor.get_yres());
    } else {
      const auto out_filename = file["sensor_video_filename"].as<std::string>();
      _sensor_writer = std::make_unique<MovieWriter>(
          out_filename, _sensor.get_xres(), _sensor.get_yres());
    }
  } else {
    _make_sensor_video = false;
  }

  // get total sim time
  if (!file["T"]) {
    std::cout << "Warning: Didn't specify T in " << sim_file_name
              << " using default value of 5 seconds" << std::endl;
    _T = 5;
  } else {
    _T = file["T"].as<double>();
  }

  // get fps of sensor and observer
  if (!file["fps"]) {
    std::cout << "Warning: Didn't specify fps in " << sim_file_name
              << " using default value of 25 fps" << std::endl;
    _fps = 25;
  } else {
    _fps = file["fps"].as<unsigned>();
  }

  // get initial state
  if (file["init_table_pose"]) {
    TablePose init_pose = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    auto table_pose_node = file["init_table_pose"];
    if (table_pose_node["x"]) {
      init_pose.x = table_pose_node["x"].as<double>();
    }
    if (table_pose_node["y"]) {
      init_pose.y = table_pose_node["y"].as<double>();
    }
    if (table_pose_node["z"]) {
      init_pose.z = table_pose_node["z"].as<double>();
    }
    if (table_pose_node["xdot"]) {
      init_pose.xdot = table_pose_node["xdot"].as<double>();
    }
    if (table_pose_node["ydot"]) {
      init_pose.ydot = table_pose_node["ydot"].as<double>();
    }
    if (table_pose_node["zdot"]) {
      init_pose.zdot = table_pose_node["zdot"].as<double>();
    }
    if (table_pose_node["theta_x"]) {
      init_pose.theta_x = table_pose_node["theta_x"].as<double>();
    }
    if (table_pose_node["theta_y"]) {
      init_pose.theta_y = table_pose_node["theta_y"].as<double>();
    }
    if (table_pose_node["theta_dot_x"]) {
      init_pose.theta_dot_x = table_pose_node["theta_dot_x"].as<double>();
    }
    if (table_pose_node["theta_dot_y"]) {
      init_pose.theta_dot_y = table_pose_node["theta_dot_y"].as<double>();
    }
    _env.set_table_pose(init_pose);
  }

  if (file["init_ball_pose"]) {
    Eigen::Vector3d init_aor{0, 1, 0};
    BallPose init_pose = {0, 0, 0, 0, 0, 0, init_aor, 0};

    auto ball_pose_node = file["init_ball_pose"];
    if (ball_pose_node["x"]) {
      init_pose.x = ball_pose_node["x"].as<double>();
    }
    if (ball_pose_node["y"]) {
      init_pose.y = ball_pose_node["y"].as<double>();
    }
    if (ball_pose_node["z"]) {
      init_pose.z = ball_pose_node["z"].as<double>();
    }
    if (ball_pose_node["xdot"]) {
      init_pose.xdot = ball_pose_node["xdot"].as<double>();
    }
    if (ball_pose_node["ydot"]) {
      init_pose.ydot = ball_pose_node["ydot"].as<double>();
    }
    if (ball_pose_node["zdot"]) {
      init_pose.zdot = ball_pose_node["zdot"].as<double>();
    }
    if (ball_pose_node["aor"]) {
      init_aor << ball_pose_node["aor"][0].as<double>(),
          ball_pose_node["aor"][1].as<double>(),
          ball_pose_node["aor"][2].as<double>();
    }
    if (ball_pose_node["omega"]) {
      init_pose.omega = ball_pose_node["omega"].as<double>();
    }
    _env.set_ball_pose(init_pose);
  }
}

Simulator::Simulator(const std::string &config_dir)
    : _sensor(config_dir + "/sensor.yaml"), _env(config_dir + "/env.yaml") {
  parse_render_configs(config_dir + "/render.yaml");

  // construct the correct type of controller
  auto controller_file_name = config_dir + "/controller.yaml";
  auto controller_file = YAML::LoadFile(controller_file_name);
  if (!controller_file["controller_type"]) {
    std::cout << "Did not specify controller type in " << controller_file_name
              << std::endl;
    exit(1);
  }

  auto controller_type = controller_file["controller_type"].as<std::string>();
  if (controller_type == "pid") {
    _controller = std::make_unique<PIDController>(controller_file_name);
  } else {
    std::cout << "Invalid controller type " << controller_type << std::endl;
    exit(1);
  }

  _parse_sim_configs(config_dir);
}

void Simulator::run() {
  std::vector<char> sensor_pixs(_sensor.get_pixs_size());
  std::vector<char> observer_pixs;
  if (_make_observer_video) {
    observer_pixs.resize(_observer->get_pixs_size());
  }

  // Super jank hack since the library I'm using to make the videos messes us
  // the first second of video
  if (_make_observer_video) {
    _observer->observe(_env, observer_pixs);
    for (unsigned i = 0; i < 25; i++) {
      _observer_writer->addFrame((const uint8_t *)&observer_pixs[0]);
    }
  }
  if (_make_sensor_video) {
    _sensor.observe(_env, sensor_pixs);
    for (unsigned i = 0; i < 25; i++) {
      _sensor_writer->addFrame((const uint8_t *)&sensor_pixs[0]);
    }
  }

  const unsigned nsteps = _T / _env.DT;
  const unsigned step_per_sec = 1. / _env.DT;
  Reference u;
  for (unsigned i = 0; i < nsteps; i++) {
    if (i % (step_per_sec / _fps) == 0) {
      _sensor.observe(_env, sensor_pixs);
      Eigen::Vector3d true_pos = _env.get_ball_pos();
      std::cout << "True pos: [" << true_pos[0] << ", " << true_pos[1] << "]"
                << std::endl;
      _controller->react(sensor_pixs, u, _sensor);

      if (_make_observer_video) {
        _observer->observe(_env, observer_pixs);
        _observer_writer->addFrame((const uint8_t *)&observer_pixs[0]);
      }
      if (_make_sensor_video) {
        // sensor has already observered so we don't need to redo that
        _sensor_writer->addFrame((const uint8_t *)&sensor_pixs[0]);
      }
    }
    _env.step(u);
  }
}
} // namespace control_final
