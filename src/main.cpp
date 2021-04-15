#include "control_final/sensor/sensor.h"

#include "movie.h"
#include <Eigen/Dense>
#include <iostream>

using namespace control_final;
int main(int argc, char *argv[]) {

  const Eigen::Vector3d camera{0, 0, 0};
  const Eigen::Vector3d d{1, 0, 0};
  const unsigned xres = 720;
  const unsigned yres = 480;
  Sensor sensor{xres, yres, camera, d};

  Pose zeros{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  State state{zeros, zeros, zeros};
  state.ball_pose.x = 4;
  state.ball_pose.y = -3;

  MovieWriter writer("test", xres, yres);

  std::vector<char> pixs(xres * yres * 3);
  const unsigned nframes = 300;
  for (unsigned i = 0; i < nframes; i++) {
    state.ball_pose.y += 6. / nframes;
    sensor.render(state, pixs);

    writer.addFrame((const uint8_t *) &pixs[0]);
  }
}
