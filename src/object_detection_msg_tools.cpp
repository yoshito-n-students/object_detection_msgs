#include <nodelet/nodelet.h>
#include <object_detection_msgs/non_maximum_suppressor.hpp>
#include <object_detection_msgs/object_integrator.hpp>
#include <object_detection_msgs/synchronized_object_publisher.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(object_detection_msgs::NonMaximumSuppressor, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(object_detection_msgs::ObjectIntegrator, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(object_detection_msgs::SynchronizedObjectPublisher, nodelet::Nodelet);