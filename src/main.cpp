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
  constexpr unsigned xres = 720;
  constexpr unsigned yres = 480;
  /* constexpr unsigned xres = 1920; */
  /* constexpr unsigned yres = 1080; */
  Sensor sensor{xres, yres, camera, d};
  Environment env;

  /* MovieWriter writer("test", xres, yres); */

  constexpr TablePose reference{0, 0, -0.04, 0, 0, 0, 0, 0, 0, 0};
  Reference u{.table_pose = reference};

  const Eigen::Vector3d init_aor{0, 1, 0};
  const BallPose init_pos{0, 0, 0, 0.04, 0, 0, init_aor, 0.};
  env.set_ball_pose(init_pos);

  std::vector<char> pixs(xres * yres * 3);
  /* constexpr double T = 4; */
  /* constexpr unsigned fps = 1000; */
  /* constexpr unsigned nframes = T * fps; */

  // Super jank hack since the library I'm using to make the videos messes us
  // the first second of video
  /* sensor.observe(env, pixs); */
  /* for (unsigned i = 0; i < 25; i++) { */
  /*   writer.addFrame((const uint8_t *)&pixs[0]); */
  /* } */

  PIDController controller;
  env.step(u);

  sensor.observe(env, pixs);
  controller.react(pixs, u);
}
