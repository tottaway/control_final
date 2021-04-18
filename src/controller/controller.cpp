#include "control_final/controller/controller.h"

#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include <iostream>
#include <vector>

namespace control_final {

State Controller::_predict_state(std::vector<char> &pixs) {
  cv::Mat im(480, 720, CV_8UC3, (uint8_t *)&pixs[0]);
  cv::Mat gray;
  cv::cvtColor(im, gray, cv::COLOR_BGR2GRAY);
  cv::medianBlur(gray, gray, 5);

  // do edge detection
  // source: https://docs.opencv.org/3.4/da/d5c/tutorial_canny_detector.html
  int lowThreshold = 0;
  const int max_lowThreshold = 100;
  const int ratio = 3;
  const int kernel_size = 3;
  cv::Mat dst, detected_edges;
  cv::blur(gray, detected_edges, cv::Size(3, 3));
  cv::Canny(detected_edges, detected_edges, lowThreshold, lowThreshold * ratio,
        kernel_size);

  // find circles
  // source: https://docs.opencv.org/3.4/d4/d70/tutorial_hough_circle.html
  std::vector<cv::Vec3f> circles;
  HoughCircles(detected_edges, circles, cv::HOUGH_GRADIENT, 1,
               detected_edges.rows, // change this value to detect circles with
                                    // different distances to each other
               100, 30, 1, 100 // change the last two parameters
               // (min_radius & max_radius) to detect larger circles
  );
  size_t ncircles = circles.size();
  std::cout << ncircles << std::endl;
  for (size_t i = 0; i < ncircles; i++) {
    cv::Vec3i c = circles[i];
    cv::Point center = cv::Point(c[0], c[1]);
    // circle center
    cv::circle(im, center, 1, cv::Scalar(0, 100, 100), 3, cv::LINE_AA);
    // circle outline
    int radius = c[2];
    cv::circle(im, center, radius, cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
  }
  cv::imshow("detected circles", im);
  cv::waitKey();
}

} // namespace control_final
