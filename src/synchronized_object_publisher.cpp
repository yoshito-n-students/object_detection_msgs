#include <nodelet/nodelet.h>
#include <object_detection_msgs/synchronized_object_publisher.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(object_detection_msgs::SynchronizedObjectPublisher, nodelet::Nodelet);