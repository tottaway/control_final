#include "control_final/controller/controller.h"
#include "control_final/sensor/sensor.h"

#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include <iostream>
#include <math.h>
#include <vector>

namespace control_final {

using namespace cv;
State Controller::_predict_state(std::vector<char> &pixs,
                                 const Sensor &sensor) {
  const unsigned xres = sensor.get_xres();
  const unsigned yres = sensor.get_yres();

  Mat im(yres, xres, CV_8UC3, (uint8_t *)&pixs[0]);

  // get grayscale version of image
  Mat gray;
  cvtColor(im, gray, COLOR_BGR2GRAY);
  medianBlur(gray, gray, 5);

  // do edge detection
  // source: https://docs.opencv.org/3.4/da/d5c/tutorial_canny_detector.html
  int lowThreshold = 0;
  const int max_lowThreshold = 100;
  const int ratio = 3;
  const int kernel_size = 3;
  Mat dst, detected_edges;
  blur(gray, detected_edges, Size(3, 3));
  Canny(detected_edges, detected_edges, lowThreshold, lowThreshold * ratio,
        kernel_size);

  // find circles
  // source: https://docs.opencv.org/3.4/d4/d70/tutorial_hough_circle.html
  std::vector<Vec3f> circles;
  const int min_radius = 1;
  // TODO: calculate second part based off of the angle the ball should take up
  // in the image based off of the distance from the table
  const int max_radius = xres / 24;
  HoughCircles(detected_edges, circles, HOUGH_GRADIENT, 1,
               detected_edges.rows, // change this value to detect circles with
                                    // different distances to each other
               100, 30, min_radius, max_radius // change the last two parameters
               // (min_radius & max_radius) to detect larger circles
  );

  size_t ncircles = circles.size();

  // assert that there is exactly one circle found
  // TODO: handle these cases more cleanly
  if (ncircles == 0) {
    std::cout << "Need to handle case where ball isn't detected" << std::endl;
    exit(1);
  }
  if (ncircles > 1) {
    std::cout << "Need to handle case where more that one circle is detected"
              << std::endl;
    exit(1);
  }

  Vec3i c = circles[0];

  // Uncomment to display image with hightlighted circle and then exit
  /* // circle center */
  /* circle(im, center, 1, Scalar(0, 100, 100), 3, LINE_AA); */
  /* // circle outline */
  /* int radius = c[2]; */
  /* circle(im, center, radius, Scalar(255, 0, 255), 3, LINE_AA); */
  /* cv::imshow("detected circles", im); */
  /* cv::waitKey(); */
  /* exit(1); */

  // We assume that the ball is on the table and that the camera is pointing
  // straight down

  // Furthermore due to the way that the raytracer works (this isn't
  // configurable at the moment) the top of the image is -x and the right of the
  // image is -y

  // The final thing we need is the fact that the viewing angle in the long
  // dimension is 0.471 radians

  // TODO: check radius of circle to sanity check
  const double dist_camera_to_table = sensor.get_camera_pos()[2];
  const double scaled_circle_center_x = (c[0] / (double)xres) - 0.5;
  const double scaled_circle_center_y = (c[1] / (double)yres) - 0.5;

  // This is the distance such that the screen has width 1
  const double scaled_dist_x = (0.5 * ((double)yres / xres) / tan(0.471 / 2));
  const double angle_x = atan(scaled_circle_center_x / scaled_dist_x);

  // This is the distance such that the screen has height 1
  const double scaled_dist_y = (0.5 / tan(0.471 / 2));
  const double angle_y = atan(scaled_circle_center_y / scaled_dist_y);

  // Find the actual coordinates
  // TODO: see if this whole calculation simplifies
  const double x_coord = dist_camera_to_table * tan(angle_x);
  const double y_coord = dist_camera_to_table * tan(angle_y);

  State ret;
  // Deal with weird axis transformation
  // TODO: zero out of calculate rest of fields
  ret.ball_pose.x = y_coord;
  ret.ball_pose.y = -x_coord;
  return ret;
}
} // namespace control_final
