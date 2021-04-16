#include "control_final/sensor/sensor.h"
#include "control_final/model/environment.h"

#include "movie.h"
#include <Eigen/Dense>
#include <iostream>

using namespace control_final;
int main(int argc, char *argv[]) {

  const Eigen::Vector3d camera{-4, 0, 0};
  const Eigen::Vector3d d{1, 0, 0};
  /* const unsigned xres = 720; */
  /* const unsigned yres = 480; */
  const unsigned xres = 1920;
  const unsigned yres = 1080;
  Sensor sensor{xres, yres, camera, d};
  Environment env;

  MovieWriter writer("test", xres, yres);

  Pose reference{0, 0, -0.5, 0, 0, 0, -0.2, 0, 0, 0, 0, 0};
  Reference u{.table_pose = reference};

  std::vector<char> pixs(xres * yres * 3);
  const double T = 4;
  const unsigned fps = 32;
  const unsigned nframes = T * fps;
  for (unsigned i = 0; i < nframes; i++) {
    u.table_pose.theta_x += 0.005;
    env.step(u);
    sensor.render(env.get_state(), pixs);

    writer.addFrame((const uint8_t *)&pixs[0]);
  }
}
