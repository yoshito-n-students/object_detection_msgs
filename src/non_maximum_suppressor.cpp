#include <nodelet/nodelet.h>
#include <object_detection_msgs/non_maximum_suppressor.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(object_detection_msgs::NonMaximumSuppressor, nodelet::Nodelet);