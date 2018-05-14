#ifndef OBJECT_DETECTION_MSGS_SYNCHRONIZED_OBJECT_PUBLISHER_HPP
#define OBJECT_DETECTION_MSGS_SYNCHRONIZED_OBJECT_PUBLISHER_HPP

#include <vector>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <image_transport/transport_hints.h>
#include <nodelet/nodelet.h>
#include <object_detection_msgs/GetObjects.h>
#include <object_detection_msgs/Objects.h>
#include <object_detection_msgs/SetObjects.h>
#include <ros/publisher.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace object_detection_msgs {

class SynchronizedObjectPublisher : public nodelet::Nodelet {
public:
  SynchronizedObjectPublisher() {}

  virtual ~SynchronizedObjectPublisher() {}

private:
  virtual void onInit() {
    ros::NodeHandle &nh(getNodeHandle());
    ros::NodeHandle &pnh(getPrivateNodeHandle());

    // load params
    const bool subscribe_image(pnh.param("subscribe_image", false));
    const bool subscribe_objects(pnh.param("subscribe_objects", false));
    objects_.names = pnh.param("names", std::vector< std::string >());
    objects_.probabilities = pnh.param("probablities", std::vector< double >());
    objects_.contours = contoursParam(pnh, "contours", std::vector< Points >());

    // services for user-specified objects
    get_server_ =
        nh.advertiseService("get_objects", &SynchronizedObjectPublisher::getObjects, this);
    set_server_ =
        nh.advertiseService("set_objects", &SynchronizedObjectPublisher::setObjects, this);

    // advertise synchronized objects
    object_publisher_ = nh.advertise< Objects >("objects_out", 1, true);

    // subscribe images and/or objects to be synchronized
    if (subscribe_image) {
      image_transport::ImageTransport it(nh);
      const image_transport::TransportHints default_hints;
      image_subscriber_ =
          it.subscribe("image_raw", 1, &SynchronizedObjectPublisher::publishObjects, this,
                       image_transport::TransportHints(default_hints.getTransport(),
                                                       default_hints.getRosHints(), pnh));
    }
    if (subscribe_objects) {
      object_subscriber_ =
          nh.subscribe("objects_in", 1, &SynchronizedObjectPublisher::mergeAndPublishObjects, this);
    }
    if (!subscribe_image && !subscribe_objects) {
      NODELET_ERROR(
          "Nothing subscribed. Either ~subscribe_image or ~subscribe_objects should be ture");
    }
  }

  bool getObjects(GetObjects::Request & /* empty */, GetObjects::Response &response) {
    response.objects = objects_;
    return true;
  }

  bool setObjects(SetObjects::Request &request, SetObjects::Response & /* empty */) {
    objects_ = request.objects;
    return true;
  }

  void publishObjects(const sensor_msgs::ImageConstPtr &image_in) {
    const ObjectsPtr objects_out(new Objects(objects_));
    objects_out->header = image_in->header;
    object_publisher_.publish(objects_out);
  }

  void mergeAndPublishObjects(const ObjectsConstPtr &objects_in) {
    const ObjectsPtr objects_out(new Objects(*objects_in));

    // complete target objects
    {
      const std::size_t max_size(
          std::max(std::max(objects_out->names.size(), objects_out->probabilities.size()),
                   objects_out->contours.size()));
      objects_out->names.resize(max_size, "");
      objects_out->probabilities.resize(max_size, -1.);
      objects_out->contours.resize(max_size, Points());
    }

    // append user-specified to target objects
    objects_out->names.insert(objects_out->names.end(), objects_.names.begin(),
                              objects_.names.end());
    objects_out->probabilities.insert(objects_out->probabilities.end(),
                                      objects_.probabilities.begin(), objects_.probabilities.end());
    objects_out->contours.insert(objects_out->contours.end(), objects_.contours.begin(),
                                 objects_.contours.end());

    object_publisher_.publish(objects_out);
  }

  // utility function to load array of image points
  // (this cannot be a static member function due to NODELET_XXX macros)
  std::vector< Points > contoursParam(ros::NodeHandle &nh, const std::string &name,
                                      const std::vector< Points > &default_val) {
    // load a parameter tree
    XmlRpc::XmlRpcValue contours_tree;
    if (!nh.getParam(name, contours_tree)) {
      return default_val;
    }

    // convert the parameter tree to value
    std::vector< Points > contours;
    try {
      for (std::size_t i = 0; i < contours_tree.size(); ++i) {
        XmlRpc::XmlRpcValue &points_tree(contours_tree[i]);
        Points points;
        for (std::size_t j = 0; j < points_tree.size(); ++j) {
          XmlRpc::XmlRpcValue &point_tree(points_tree[j]);
          Point point;
          point.x = point_tree[0];
          point.y = point_tree[1];
          points.points.push_back(point);
        }
        contours.push_back(points);
      }
    } catch (const XmlRpc::XmlRpcException &error) {
      NODELET_ERROR_STREAM("Error in parsing parameter " << nh.resolveName(name) << ": "
                                                         << error.getMessage()
                                                         << ". Will use the default value.");
      return default_val;
    }

    return contours;
  }

private:
  Objects objects_;

  image_transport::Subscriber image_subscriber_;
  ros::Subscriber object_subscriber_;
  ros::Publisher object_publisher_;
  ros::ServiceServer get_server_, set_server_;
};

} // namespace object_detection_msgs

#endif