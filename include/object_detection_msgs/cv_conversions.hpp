#ifndef OBJECT_DETECTION_MSGS_CV_CONVERSIONS_HPP
#define OBJECT_DETECTION_MSGS_CV_CONVERSIONS_HPP

#include <vector>

#include <object_detection_msgs/Objects.h>
#include <object_detection_msgs/Point.h>
#include <object_detection_msgs/Points.h>

#include <opencv2/core.hpp>

#include <boost/foreach.hpp>

namespace object_detection_msgs {

// **************************
// Conversion to OpenCV types
// **************************

template < typename T > static inline cv::Point_< T > toCvPoint_(const Point &point_msg) {
  return cv::Point_< T >(static_cast< T >(point_msg.x), static_cast< T >(point_msg.y));
}

template < typename T >
static inline std::vector< cv::Point_< T > > toCvPoints_(const Points &points_msg) {
  std::vector< cv::Point_< T > > cv_points;
  BOOST_FOREACH (const Point &point_msg, points_msg.points) {
    cv_points.push_back(toCvPoint_< T >(point_msg));
  }
  return cv_points;
}

template < typename T >
static inline std::vector< std::vector< cv::Point_< T > > >
toCvContours_(const std::vector< Points > &contours_msg) {
  std::vector< std::vector< cv::Point_< T > > > cv_contours;
  BOOST_FOREACH (const Points &points_msg, contours_msg) {
    cv_contours.push_back(toCvPoints_< T >(points_msg));
  }
  return cv_contours;
}

// ********************************************************
// Conversions to cv::Point and cv::Point2f for convenience
// ********************************************************

// cv::Point

static inline cv::Point toCvPoint(const Point &point_msg) {
  return toCvPoint_< cv::Point::value_type >(point_msg);
}

static inline std::vector< cv::Point > toCvPoints(const Points &points_msg) {
  return toCvPoints_< cv::Point::value_type >(points_msg);
}

static inline std::vector< std::vector< cv::Point > >
toCvContours(const std::vector< Points > &contours_msg) {
  return toCvContours_< cv::Point::value_type >(contours_msg);
}

// cv::Point2f

static inline cv::Point2f toCvPoint2f(const Point &point_msg) {
  return toCvPoint_< cv::Point2f::value_type >(point_msg);
}

static inline std::vector< cv::Point2f > toCvPoints2f(const Points &points_msg) {
  return toCvPoints_< cv::Point2f::value_type >(points_msg);
}

static inline std::vector< std::vector< cv::Point2f > >
toCvContours2f(const std::vector< Points > &contours_msg) {
  return toCvContours_< cv::Point2f::value_type >(contours_msg);
}

// *******************************
// Conversion to ROS message types
// *******************************

template < typename T > static inline Point toPointMsg(const T x, const T y) {
  Point point_msg;
  point_msg.x = static_cast< float >(x);
  point_msg.y = static_cast< float >(y);
  return point_msg;
}

template < typename T > static inline Point toPointMsg(const cv::Point_< T > &cv_point) {
  return toPointMsg(cv_point.x, cv_point.y);
}

template < typename T >
static inline Points toPointsMsg(const std::vector< cv::Point_< T > > &cv_points) {
  Points points_msg;
  BOOST_FOREACH (const cv::Point_< T > &cv_point, cv_points) {
    points_msg.points.push_back(toPointMsg(cv_point));
  }
  return points_msg;
}

template < typename T > static inline Points toPointsMsg(const cv::Rect_< T > &cv_rect) {
  Points points_msg;
  const cv::Point_< T > tl(cv_rect.tl()), br(cv_rect.br());
  points_msg.points.push_back(toPointMsg(tl.x, tl.y));
  points_msg.points.push_back(toPointMsg(br.x, tl.y));
  points_msg.points.push_back(toPointMsg(br.x, br.y));
  points_msg.points.push_back(toPointMsg(tl.x, br.y));
  return points_msg;
}

template < typename T > static inline Points toPointsMsg(const cv::Size_< T > &cv_size) {
  return toPointsMsg(
      cv::Rect_< T >(cv::Point_< T >(static_cast< T >(0), static_cast< T >(0)), cv_size));
}

template < typename T >
static inline std::vector< Points >
toContoursMsg(const std::vector< std::vector< cv::Point_< T > > > &cv_contours) {
  std::vector< Points > contours_msg;
  BOOST_FOREACH (const std::vector< cv::Point_< T > > &cv_points, cv_contours) {
    contours_msg.push_back(toPointsMsg(cv_points));
  }
  return contours_msg;
}

} // namespace object_detection_msgs

#endif