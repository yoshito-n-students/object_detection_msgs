#ifndef OBJECT_DETECTION_MSGS_CV_CONVERSIONS_HPP
#define OBJECT_DETECTION_MSGS_CV_CONVERSIONS_HPP

#include <vector>

#include <object_detection_msgs/Objects.h>
#include <object_detection_msgs/Point.h>
#include <object_detection_msgs/Points.h>

#include <opencv2/core.hpp>

#include <boost/foreach.hpp>

namespace object_detection_msgs {

static inline cv::Point toCvPoint(const Point &point_msg) {
  return cv::Point(point_msg.x, point_msg.y);
}

static inline std::vector< cv::Point > toCvPoints(const Points &points_msg) {
  std::vector< cv::Point > cv_points;
  BOOST_FOREACH (const Point &point_msg, points_msg.points) {
    cv_points.push_back(toCvPoint(point_msg));
  }
  return cv_points;
}

static inline std::vector< std::vector< cv::Point > >
toCvContours(const std::vector< Points > &contours_msg) {
  std::vector< std::vector< cv::Point > > cv_contours;
  BOOST_FOREACH (const Points &points_msg, contours_msg) {
    cv_contours.push_back(toCvPoints(points_msg));
  }
  return cv_contours;
}

static inline Point toPointMsg(const cv::Point &cv_point) {
  Point point_msg;
  point_msg.x = cv_point.x;
  point_msg.y = cv_point.y;
  return point_msg;
}

static inline Points toPointsMsg(const std::vector< cv::Point > &cv_points) {
  Points points_msg;
  BOOST_FOREACH (const cv::Point &cv_point, cv_points) {
    points_msg.points.push_back(toPointMsg(cv_point));
  }
  return points_msg;
}

static inline std::vector< Points >
toContoursMsg(const std::vector< std::vector< cv::Point > > &cv_contours) {
  std::vector< Points > contours_msg;
  BOOST_FOREACH (const std::vector< cv::Point > &cv_points, cv_contours) {
    contours_msg.push_back(toPointsMsg(cv_points));
  }
  return contours_msg;
}

} // namespace object_detection_msgs

#endif