#ifndef OBJECT_DETECTION_MSGS_NON_MAXIMUM_SUPPRESSOR_HPP
#define OBJECT_DETECTION_MSGS_NON_MAXIMUM_SUPPRESSOR_HPP

#include <functional> // for std::greater<>
#include <iterator>   // for std::advance()
#include <map>
#include <string>
#include <utility> // for std::make_pair()
#include <vector>

#include <nodelet/nodelet.h>
#include <object_detection_msgs/Objects.h>
#include <object_detection_msgs/Points.h>
#include <object_detection_msgs/cv_conversions.hpp>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <boost/foreach.hpp>

namespace object_detection_msgs {

// calc overlap of 2 contours
static inline double computeOverlap(const std::vector< cv::Point > &a,
                                    const std::vector< cv::Point > &b) {
  // rectangle of interest
  const cv::Rect roi(cv::boundingRect(a) & cv::boundingRect(b));
  const cv::Point offset(-roi.tl());

  // draw a polygon represented by contour A
  cv::Mat poly_a(cv::Mat::zeros(roi.size(), CV_8UC1));
  cv::fillPoly(poly_a, std::vector< std::vector< cv::Point > >(1, a), 1, cv::LINE_8, 1, offset);

  // draw a polygon represented by contour B
  cv::Mat poly_b(cv::Mat::zeros(roi.size(), CV_8UC1));
  cv::fillPoly(poly_b, std::vector< std::vector< cv::Point > >(1, b), 1, cv::LINE_8, 1, offset);

  // calc <area of polygon A and B> / <area of polygon A or B>
  //    (0: no overlap, 1: complete overlap)
  return static_cast< double >(cv::countNonZero(poly_a & poly_b)) /
         cv::countNonZero(poly_a | poly_b);
}

// variant of cv::dnn::NMSBoxes() for general contours
static inline void NMSContours(const std::vector< std::vector< cv::Point > > &contours,
                               const std::vector< double > &scores, const double score_threshold,
                               const double nms_threshold, std::vector< int > &indices,
                               const double eta = 1., const int top_k = 0) {
  // sort scores (with corresponding indices)
  typedef std::multimap< double, int, std::greater< double > > ScoreMap;
  ScoreMap score_map;
  for (std::size_t i = 0; i < std::min(scores.size(), contours.size()); ++i) {
    // validate score
    const double score(scores[i]);
    if (score < 0. || score < score_threshold || score > 1.) {
      continue;
    }
    // validate contour associated to score
    if (contours[i].empty()) {
      continue;
    }
    score_map.insert(std::make_pair(score, i));
  }

  // keep top_k scores if needed.
  if (top_k > 0 && top_k < score_map.size()) {
    ScoreMap::iterator erase_begin(score_map.begin());
    std::advance(erase_begin, top_k);
    score_map.erase(erase_begin, score_map.end());
  }

  // do nms.
  double adaptive_threshold(nms_threshold);
  indices.clear();
  BOOST_FOREACH (const ScoreMap::value_type &score_pair, score_map) {
    const int idx(score_pair.second);
    bool keep(true);
    BOOST_FOREACH (const int kept_idx, indices) {
      if (computeOverlap(contours[idx], contours[kept_idx]) > adaptive_threshold) {
        keep = false;
        break;
      }
    }
    if (keep) {
      indices.push_back(idx);
      if (eta < 1. && adaptive_threshold > 0.5) {
        adaptive_threshold *= eta;
      }
    }
  }
}

class NonMaximumSuppressor : public nodelet::Nodelet {
public:
  NonMaximumSuppressor() {}
  virtual ~NonMaximumSuppressor() {}

private:
  virtual void onInit() {
    ros::NodeHandle &nh(getNodeHandle());
    ros::NodeHandle &pnh(getPrivateNodeHandle());

    score_threshold_ = pnh.param("score_threshold", 0.4);
    nms_threshold_ = pnh.param("nms_threshold", 0.5);
    eta_ = pnh.param("eta", 1.);
    top_k_ = pnh.param("top_k", 0);

    publisher_ = nh.advertise< Objects >("objects_out", 1, true);
    subscriber_ = nh.subscribe("objects_in", 1, &NonMaximumSuppressor::suppress, this);
  }

  void suppress(const ObjectsConstPtr &object_in) {
    // non maximum suppression
    std::vector< int > indices;
    NMSContours(toCvContours(object_in->contours), object_in->probabilities, score_threshold_,
                nms_threshold_, indices, eta_, top_k_);

    // pick kept objects
    const ObjectsPtr object_out(new Objects);
    object_out->header.stamp = object_in->header.stamp;
    BOOST_FOREACH (const int idx, indices) {
      object_out->names.push_back(idx < object_in->names.size() ? object_in->names[idx]
                                                                : std::string(""));
      object_out->contours.push_back(idx < object_in->contours.size() ? object_in->contours[idx]
                                                                      : Points());
      object_out->probabilities.push_back(
          idx < object_in->probabilities.size() ? object_in->probabilities[idx] : -1.);
    }

    // publish kept objects
    publisher_.publish(object_out);
  }

private:
  double score_threshold_, nms_threshold_, eta_;
  int top_k_;

  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
};
} // namespace object_detection_msgs

#endif