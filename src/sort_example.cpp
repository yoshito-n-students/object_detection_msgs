#include <functional> // for less<>, greater<>
#include <string>

#include <object_detection_msgs/Objects.h>
#include <object_detection_msgs/Points.h>
#include <object_detection_msgs/cv_conversions.hpp>
#include <object_detection_msgs/sort.hpp>
#include <ros/console.h>

#include <opencv2/imgproc.hpp>

namespace odm = object_detection_msgs;

odm::Points triangleContour(const double len) {
  odm::Points contour;
  contour.points.resize(3);
  contour.points[0].x = 0;
  contour.points[0].y = 0;
  contour.points[1].x = 0;
  contour.points[1].y = static_cast<int>(len);
  contour.points[2].x = static_cast<int>(len * 0.5 /* cos(pi/3) */);
  contour.points[2].y = static_cast<int>(len * 0.8660 /* sin(pi/3) */);
  return contour;
}

struct GreaterArea {
  bool operator()(const odm::Points &a, const odm::Points &b) const {
    return cv::contourArea(odm::toCvPoints(a)) > cv::contourArea(odm::toCvPoints(b));
  }
};

int main(int argc, char *argv[]) {
  // ****************
  // Triangle objects
  // ****************

  odm::Objects objs;
  //
  objs.names.push_back("smallest contour with medium probability");
  objs.probabilities.push_back(0.5);
  objs.contours.push_back(triangleContour(5.));
  //
  objs.names.push_back("2nd smallest contour with smallest probability");
  objs.probabilities.push_back(0.);
  objs.contours.push_back(triangleContour(10.));
  //
  objs.names.push_back("medium contour with 2nd smallest probability");
  objs.probabilities.push_back(0.2);
  objs.contours.push_back(triangleContour(15.));
  //
  objs.names.push_back("2nd largest contour with largest probability");
  objs.probabilities.push_back(0.9);
  objs.contours.push_back(triangleContour(20.));
  //
  objs.names.push_back("largest contour with 2nd largest probability");
  objs.probabilities.push_back(0.8);
  objs.contours.push_back(triangleContour(25.));

  // *****************************
  // Sorting by different policies
  // *****************************

  ROS_INFO_STREAM("Before sorting:\n" << objs);

  odm::sortByName(&objs, std::less<std::string>());
  ROS_INFO_STREAM("Sorted by name:\n" << objs);

  odm::sortByProbability(&objs, std::greater<double>());
  ROS_INFO_STREAM("Sorted by probability:\n" << objs);

  odm::sortByContour(&objs, GreaterArea());
  ROS_INFO_STREAM("Sorted by contour area:\n" << objs);

  return 0;
}