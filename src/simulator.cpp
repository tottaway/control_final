#include "control_final/simulator.h"
#include "control_final/controller/controller.h"
#include "control_final/controller/pid_controller.h"
#include "control_final/controller/table_step.h"
#include "control_final/model/environment.h"
#include "control_final/model/render_config.h"
#include "control_final/model/state.h"
#include "control_final/sensor/sensor.h"
#include "control_final/util/parse_configs.h"

#include "yaml-cpp/yaml.h"
#include <Eigen/Dense>
#include <iostream>
#include <map>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace control_final {

void Simulator::init_logs() {
  // open all of the enabled logs and write headers
  if (m_env_logging_enabled) {
    m_env_log.open(m_env_log_filename);
    m_env_log << "t,ball_x,ball_y,ball_z,ball_xdot,ball_ydot,ball_zdot,"
                 "ball_aor_x,ball_aor_y,ball_aor_z,ball_omega,table_x,"
                 "table_y,table_z,table_xdot,table_ydot,table_zdot,table_theta_x,"
                 "table_theta_y,table_theta_dot_x,table_theta_dot_y"
              << std::endl;
  }
  if (m_sensor_logging_enabled) {
    m_sensor_log.open(m_sensor_log_filename);
    m_sensor_log << "t,ball_x,ball_y" << std::endl;
  }
  if (m_controller_logging_enabled) {
    m_controller_log.open(m_controller_log_filename);
    m_controller_log << "t,torque_x,torque_y,ref_theta_x,ref_theta_y"
                     << std::endl;
  }
}

void Simulator::close_logs() {
  if (m_env_logging_enabled)
    m_env_log.close();
  if (m_sensor_logging_enabled)
    m_sensor_log.close();
  if (m_controller_logging_enabled)
    m_controller_log.close();
}

void Simulator::write_env_log(const double t) {
  if (!m_env_logging_enabled)
    return;

  m_env_log << t << ",";

  const auto ball_pos = m_env.get_ball_pos();
  m_env_log << ball_pos[0] << "," << ball_pos[1] << "," << ball_pos[2] << ",";

  const auto ball_vel = m_env.get_ball_vel();
  m_env_log << ball_vel[0] << "," << ball_vel[1] << "," << ball_vel[2] << ",";

  const auto ball_aor = m_env.get_ball_aor();
  m_env_log << ball_aor[0] << "," << ball_aor[1] << "," << ball_aor[2] << ",";

  m_env_log << m_env.get_ball_omega() << ",";

  const auto table_pos = m_env.get_table_pos();
  m_env_log << table_pos[0] << "," << table_pos[1] << "," << table_pos[2] << ",";

  const auto table_vel = m_env.get_table_vel();
  m_env_log << table_vel[0] << "," << table_vel[1] << "," << table_vel[2] << ",";

  const auto table_rot = m_env.get_table_rot();
  m_env_log << table_rot[0] << "," << table_rot[1] << ",";

  const auto table_rot_vel = m_env.get_table_rot_vel();
  m_env_log << table_rot_vel[0] << "," << table_rot_vel[1];

  m_env_log << std::endl;
}

void Simulator::write_sensor_log(const double t) {
  if (!m_sensor_logging_enabled)
    return;
  m_sensor_log << t << ",";
  m_sensor_log << m_controller->get_curr_x() << "," << m_controller->get_curr_y();
  m_sensor_log << std::endl;
}

void Simulator::write_controller_log(const double t,
                                     const ControllerOutput &u) {
  if (!m_controller_logging_enabled)
    return;
  m_controller_log << t << ",";

  m_controller_log << u.torque_x << "," << u.torque_y << ",";

  m_controller_log << m_controller->get_ref_theta_x() << ","
                   << m_controller->get_ref_theta_y();

  m_controller_log << std::endl;
}

void Simulator::parse_sim_configs(const YAML::Node &node) {
  auto sim_node = node["sim"];

  if (node["log_env"]) {
    m_env_logging_enabled = true;
    m_env_log_filename = node["log_env"].as<std::string>();
  }

  if (node["log_sensor"]) {
    m_sensor_logging_enabled = true;
    m_sensor_log_filename = node["log_sensor"].as<std::string>();
  }

  if (node["log_controller"]) {
    m_controller_logging_enabled = true;
    m_controller_log_filename = node["log_controller"].as<std::string>();
  }

  // get fps of sensor and observer
  if (!sim_node["fps"]) {
    std::cout
        << "Warning: Didn't specify fps in sim, using default value of 25 fps"
        << std::endl;
    m_fps = 25;
  } else {
    m_fps = sim_node["fps"].as<unsigned>();
  }

  // get information related to observer
  if (sim_node["make_observer_video"] &&
      sim_node["make_observer_video"].as<bool>()) {
    m_make_observer_video = true;

    // This seems pretty gross but I want to make it optional to provide config
    // file for the observer so I need to check the sim config file before
    // constructing the observer which means I can't do this in the initializer
    // list. There is probably a better way of doing this
    m_observer = std::make_unique<Sensor>(node, "observer");

    // have sensible default for output file
    if (!sim_node["observer_video_filename"]) {
      cv::Size frame_size{m_observer->get_xres(), m_observer->get_yres()};
      m_observer_writer = std::make_unique<cv::VideoWriter>(
          "observer_out.mp4", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'),
          (double)m_fps, frame_size, true);
    } else {
      const auto out_filename =
          sim_node["observer_video_filename"].as<std::string>();

      cv::Size frame_size{m_observer->get_xres(), m_observer->get_yres()};
      m_observer_writer = std::make_unique<cv::VideoWriter>(
          out_filename, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'),
          (double)m_fps, frame_size, true);
    }
  } else {
    m_make_observer_video = false;
  }

  // give support for saving sensor video
  if (sim_node["make_sensor_video"] &&
      sim_node["make_sensor_video"].as<bool>()) {
    m_make_sensor_video = true;
    const auto sensor_node = node["sensor"];

    // have sensible default for output file
    if (!sim_node["sensor_video_filename"]) {
      cv::Size frame_size{m_sensor.get_xres(), m_sensor.get_yres()};
      m_sensor_writer = std::make_unique<cv::VideoWriter>(
          "sensor_out.mp4", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'),
          (double)m_fps, frame_size, true);
    } else {
      const auto out_filename =
          sim_node["sensor_video_filename"].as<std::string>();
      cv::Size frame_size{m_sensor.get_xres(), m_sensor.get_yres()};
      m_sensor_writer = std::make_unique<cv::VideoWriter>(
          out_filename, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'),
          (double)m_fps, frame_size, true);
    }
  } else {
    m_make_sensor_video = false;
  }

  // get total sim time
  if (!sim_node["T"]) {
    std::cout
        << "Warning: Didn't specify T in sim, using default value of 5 seconds"
        << std::endl;
    m_T = 5;
  } else {
    m_T = sim_node["T"].as<double>();
  }

  // get initial state
  if (sim_node["init_table_pose"]) {
    TablePose init_pose = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    auto table_pose_node = sim_node["init_table_pose"];
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
    m_env.set_table_pose(init_pose);
  }

  if (sim_node["init_ball_pose"]) {
    Eigen::Vector3d init_aor{0, 1, 0};
    BallPose init_pose = {0, 0, 0, 0, 0, 0, init_aor, 0};

    auto ball_pose_node = sim_node["init_ball_pose"];
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
      init_pose.axis_of_rotation = init_aor;
    }
    if (ball_pose_node["omega"]) {
      init_pose.omega = ball_pose_node["omega"].as<double>();
    }
    m_env.set_ball_pose(init_pose);
  }
}

Simulator::Simulator(const YAML::Node &node)
    : m_sensor(node, "sensor"), m_env(node) {
  // construct the correct type of controller
  if (!node["controller"] || !node["controller"]["controller_type"]) {
    std::cout << "Did not specify controller type in config file" << std::endl;
    exit(1);
  }

  const auto controller_type =
      node["controller"]["controller_type"].as<std::string>();

  if (controller_type == "pid") {
    m_controller = std::make_unique<PIDController>(node);
  } else if (controller_type == "table_step") {
    m_controller = std::make_unique<TableStepController>(node);
  } else {
    std::cout << "Invalid controller type " << controller_type << std::endl;
    exit(1);
  }

  parse_sim_configs(node);
}

void Simulator::run() {
  init_logs();

  std::vector<char> sensor_pixs(m_sensor.get_pixs_size());
  std::vector<char> observer_pixs;
  if (m_make_observer_video) {
    observer_pixs.resize(m_observer->get_pixs_size());
  }

  const unsigned nsteps = m_T / m_env.dt;
  const unsigned step_per_sec = 1. / m_env.dt;
  ControllerOutput u;
  m_sensor.observe(m_env, sensor_pixs);
  m_controller->react(sensor_pixs, m_sensor, 0);
  double t = 0;
  // TODO: get a progress bar for this
  for (unsigned i = 0; i < nsteps; i++) {
    if (i % (step_per_sec / m_fps) == 0) {
      m_sensor.observe(m_env, sensor_pixs);
      m_controller->react(sensor_pixs, m_sensor, t);
      write_sensor_log(t);

      if (m_make_observer_video) {
        m_observer->observe(m_env, observer_pixs);
        cv::Mat im(m_observer->get_yres(), m_observer->get_xres(), CV_8UC3,
                   (uint8_t *)&observer_pixs[0]);
        m_observer_writer->write(im);
      }
      if (m_make_sensor_video) {
        // sensor has already observered so we don't need to redo that
        cv::Mat im(m_sensor.get_yres(), m_sensor.get_xres(), CV_8UC3,
                   (uint8_t *)&sensor_pixs[0]);
        m_sensor_writer->write(im);
      }
    }
    m_controller->step(u, m_env.get_table_pose());

    write_controller_log(t, u);
    write_env_log(t);

    m_env.step(u);

    t += m_env.dt;
  }
  if (m_observer_writer)
    m_observer_writer->release();
  if (m_sensor_writer)
    m_sensor_writer->release();

  close_logs();
}
} // namespace control_final
