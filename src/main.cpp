#include "control_final/sensor/sensor.h"
#include "control_final/model/environment.h"
#include "control_final/controller/pid_controller.h"

#include "movie.h"
#include <Eigen/Dense>
#include <iostream>

using namespace control_final;
int main(int argc, char *argv[]) {

  /* const Eigen::Vector3d camera{-2, 0, 0.3}; */
  /* const Eigen::Vector3d d{1, 0, -0.1}; */
  const Eigen::Vector3d camera{0, 0, 3};
  const Eigen::Vector3d d{0, 0, -1};
  const unsigned xres = 720;
  const unsigned yres = 480;
  /* const unsigned xres = 1920; */
  /* const unsigned yres = 1080; */
  Sensor sensor{xres, yres, camera, d};
  Environment env;

  MovieWriter writer("test", xres, yres);

  TablePose reference{0, 0, -0.04, 0, 0, 0, 0, 0, 0, 0};
  Reference u{.table_pose = reference};

  Eigen::Vector3d init_aor{0, 1, 0};
  BallPose init_pos{0, 0, 0, 0.04, 0, 0, init_aor, 0.};
  env.set_ball_pose(init_pos);

  std::vector<char> pixs(xres * yres * 3);
  const double T = 4;
  const unsigned fps = 1000;
  const unsigned nframes = T * fps;
  /* Eigen::Vector3d init_pos{0, 0, 1}; */
  /* env.set_ball_pos(init_pos); */


  // Super jank hack since the library I'm using to make the videos messes us
  // the first second of video
  sensor.observe(env.get_state(), pixs);
  for (unsigned i = 0; i < 25; i++) {
    writer.addFrame((const uint8_t *)&pixs[0]);
  }

  PIDController controller;
  for (unsigned i = 0; i < nframes; i++) {
    /* u.table_pose.theta_x = sin(i / 200.) / 50.; */
    /* u.table_pose.theta_y = cos(i / 200.) / 50.; */
    env.step(u);

    sensor.observe(env.get_state(), pixs);
    controller.react(pixs, u);
  }
}
