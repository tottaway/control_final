#include "control_final/sensor/sensor.h"
#include "control_final/model/environment.h"

#include "movie.h"
#include <Eigen/Dense>
#include <iostream>

using namespace control_final;
int main(int argc, char *argv[]) {

  /* const Eigen::Vector3d camera{-3, 0, 0.1}; */
  /* const Eigen::Vector3d d{1, 0, -0.03}; */
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

  Eigen::Vector3d init_aor{0, 0, 1};
  BallPose init_pos{0, 0, 0, 0.04, 0, 0, init_aor, 10};
  env.set_ball_pose(init_pos);

  std::vector<char> pixs(xres * yres * 3);
  const double T = 4;
  const unsigned fps = 1000;
  const unsigned nframes = T * fps;
  /* Eigen::Vector3d init_pos{0, 0, 1}; */
  /* env.set_ball_pos(init_pos); */


  // Super jank hack since the library I'm using to make the videos messes us
  // the first second of video
  sensor.render(env.get_state(), pixs);
  for (unsigned i = 0; i < 25; i++) {
    writer.addFrame((const uint8_t *)&pixs[0]);
  }

  for (unsigned i = 0; i < nframes; i++) {
    /* u.table_pose.theta_x = sin(i / 200.) / 50.; */
    /* u.table_pose.theta_y = cos(i / 200.) / 50.; */
    env.step(u);

    if (i % (fps / 25) == 0) {
      sensor.render(env.get_state(), pixs);
      writer.addFrame((const uint8_t *)&pixs[0]);
    }
  }
}
