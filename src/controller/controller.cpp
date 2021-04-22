#include "control_final/controller/controller.h"
#include "control_final/model/state.h"
#include "control_final/sensor/sensor.h"

#include "kalman.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <memory>
#include <vector>

namespace control_final {

Controller::Controller(const YAML::Node &node) {
  auto controller_node = node["controller"];
  m_history_len = controller_node["history_len"].as<unsigned>();

  m_dt = 1 / node["sim"]["fps"].as<double>();

  const Eigen::Vector4d zeros{0, 0, 0, 0};
  m_history.push_front(zeros);
  m_history.push_front(zeros);

  //  A - System dynamics matrix
  //  C - Output matrix
  //  Q - Process noise covariance
  //  R - Measurement noise covariance
  //  P - Estimate error covariance

  // Most simple possible dynamics
  Eigen::Matrix4d A;
  A << 1, 0, m_dt, 0, 0, 1, 0, m_dt, 0, 0, 1, 0, 0, 0, 0, 1;

  Eigen::Matrix4d C;
  C << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

  // Not sure how to populate this one?
  Eigen::Matrix4d Q;
  Q << 1e-6, 0, 0, 0, 0, 1e-6, 0, 0, 0, 0, 1e-6, 0, 0, 0, 0, 1e-6;

  // TODO: check this experimentally
  Eigen::Matrix4d R;
  R <<
    1.15e-4, 1.1e-5, 0, 0,
    1.09e-5, 8.1e-05, 0, 0,
    0, 0, 0.01, 0,
    0, 0, 0, 0.01;

  // TODO: apparently this will converge and I can use that
  Eigen::Matrix4d P;
  P << 100, 0, 0, 0, 0, 100, 0, 0, 0, 0, 100, 0, 0, 0, 0, 100;

  m_filter = std::make_unique<KalmanFilter>(m_dt, A, C, Q, R, P);
  m_filter_initialized = false;
}

void Controller::step(ControllerOutput &u, TablePose table_pose) {
  // get the current and previous states
  const double curr_theta_x = table_pose.theta_x;
  const double curr_theta_y = table_pose.theta_y;

  const double curr_err_x = m_ref_theta_x - curr_theta_x;
  const double curr_err_y = m_ref_theta_y - curr_theta_y;

  const int curr_err_x_sign = (curr_err_x > 0) - (curr_err_x < 0);
  const int curr_err_y_sign = (curr_err_y > 0) - (curr_err_y < 0);
  const double d_err_x = -curr_err_x_sign * table_pose.theta_dot_x;
  const double d_err_y = -curr_err_y_sign * table_pose.theta_dot_y;

  // Note that rotating around the x axis affects the y coord of the ball
  // TODO: implement windup
  u.torque_x = m_inner_Kp * curr_err_x + m_inner_Kd * d_err_x;
  u.torque_y = m_inner_Kp * curr_err_y + m_inner_Kd * d_err_y;
}

using namespace cv;
void Controller::predict_state(std::vector<char> &pixs, const Sensor &sensor) {
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
  Canny(detected_edges, detected_edges, 0, 100, kernel_size);
  /* cv::imshow("detected edges", detected_edges); */
  /* cv::waitKey(); */

  // find circles
  // source: https://docs.opencv.org/3.4/d4/d70/tutorial_hough_circle.html
  std::vector<Vec3f> circles;
  const int min_radius = 1;
  // TODO: calculate second part based off of the angle the ball should take up
  // in the image based off of the distance from the table
  const int max_radius = xres / 5;
  HoughCircles(detected_edges, circles, HOUGH_GRADIENT, 1,
               detected_edges.rows, // change this value to detect circles with
                                    // different distances to each other
               200, 20, min_radius, max_radius // change the last two parameters
               // (min_radius & max_radius) to detect larger circles
  );

  size_t ncircles = circles.size();

  // assert that there is exactly one circle found
  // TODO: handle these cases more cleanly
  State ret;
  if (ncircles == 1) {
    Vec3i c = circles[0];

    // We assume that the ball is on the table and that the camera is pointing
    // straight down

    // Furthermore due to the way that the raytracer works (this isn't
    // configurable at the moment) the top of the image is -x and the right of
    // the image is -y

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

    // Deal with weird axis transformation
    // TODO: zero out of calculate rest of fields
    ret.ball_pose.x = y_coord;
    ret.ball_pose.y = -x_coord;

    // TODO: table radius should be read from config file
    ret.ball_pose.x = std::max(-0.4, ret.ball_pose.x);
    ret.ball_pose.x = std::min(0.4, ret.ball_pose.x);
    ret.ball_pose.y = std::max(-0.4, ret.ball_pose.y);
    ret.ball_pose.y = std::min(0.4, ret.ball_pose.y);
  } else if (ncircles > 1) {
    std::cout << "Need to handle case where more that one circle is detected"
              << std::endl;
    exit(1);
  }

  Eigen::Vector4d x{ret.ball_pose.x, ret.ball_pose.y, 0, 0};
  if (!m_filter_initialized) {
    m_filter->init(0, x);
    m_filter_initialized = true;
  } else {
    x[2] = (x[0] - m_history[0][0]) / m_dt;
    x[3] = (x[1] - m_history[0][1]) / m_dt;
    m_filter->update(x);
  }

  m_history.push_front(m_filter->state());
  if (m_history.size()) {
    m_history.pop_back();
  }
}
} // namespace control_final
