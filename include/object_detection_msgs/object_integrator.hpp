#ifndef OBJECT_DETECTION_MSGS_OBJECT_INTEGRATOR_HPP
#define OBJECT_DETECTION_MSGS_OBJECT_INTEGRATOR_HPP

#include <iterator> // for std::back_inserter()
#include <memory>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nodelet/nodelet.h>
#include <object_detection_msgs/Objects.h>
#include <ros/publisher.h>

#include <boost/preprocessor/enum.hpp>
#include <boost/preprocessor/enum_params.hpp>
#include <boost/preprocessor/repeat_from_to.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/range/algorithm/max_element.hpp>

namespace object_detection_msgs {

class ObjectIntegrator : public nodelet::Nodelet {
public:
  ObjectIntegrator() {}

  virtual ~ObjectIntegrator() {}

private:
  virtual void onInit() {
    ros::NodeHandle &nh = getNodeHandle();
    ros::NodeHandle &pnh = getPrivateNodeHandle();

    const int n = pnh.param("n", 2);
    const int queue_size = pnh.param("queue_size", 10);
    const std::string sync_policy = pnh.param<std::string>("sync_policy", "exact_time");

    pub_ = nh.advertise<Objects>("objects_out", 1, true);

    for (int i = 0; i < n; ++i) {
      subs_.push_back(new message_filters::Subscriber<Objects>(
          nh, "objects" + boost::lexical_cast<std::string>(i), 1));
    }

#define ODM_MSG(z, N, nil) Objects
#define ODM_SUBS_N(z, N, nil) subs_[N]
#define ODM_CASE_N(z, N, nil)                                                                      \
  case N:                                                                                          \
    if (sync_policy == "exact_time") {                                                             \
      using ExactPolicy =                                                                          \
          message_filters::sync_policies::ExactTime<BOOST_PP_ENUM(N, ODM_MSG, nil)>;               \
      using Synchronizer = message_filters::Synchronizer<ExactPolicy>;                             \
      const std::shared_ptr<Synchronizer> sync = std::make_shared<Synchronizer>(                   \
          ExactPolicy(queue_size), BOOST_PP_ENUM(N, ODM_SUBS_N, nil));                             \
      sync->registerCallback(&ObjectIntegrator::integrate##N, this);                               \
      sync_ = sync;                                                                                \
    } else if (sync_policy == "approximate_time") {                                                \
      using ApproxPolicy =                                                                         \
          message_filters::sync_policies::ApproximateTime<BOOST_PP_ENUM(N, ODM_MSG, nil)>;         \
      using Synchronizer = message_filters::Synchronizer<ApproxPolicy>;                            \
      const std::shared_ptr<Synchronizer> sync = std::make_shared<Synchronizer>(                   \
          ApproxPolicy(queue_size), BOOST_PP_ENUM(N, ODM_SUBS_N, nil));                            \
      sync->registerCallback(&ObjectIntegrator::integrate##N, this);                               \
      sync_ = sync;                                                                                \
    } else {                                                                                       \
      NODELET_ERROR_STREAM("ObjectIntegrator::onInit(): Unknown sync policy (" << sync_policy      \
                                                                               << ")");            \
    }                                                                                              \
    break;
    switch (n) {
      // expands to case 2, ..., case 8.
      // case 9 does not compile because of a bug in Synchronizer::registerCallback()
      // (https://github.com/ros/ros_comm/issues/2154).
      BOOST_PP_REPEAT_FROM_TO(2, 9, ODM_CASE_N, nil)
    default:
      NODELET_ERROR_STREAM("ObjectIntegrator::onInit(): Invalid number of subscribed topics ("
                           << n << ")");
      return;
    }
#undef ODM_MSG
#undef ODM_SUBS_N
#undef ODM_CASE_N
  }

private:
#define ODM_DEFINE_INTEGRATE_N(z, N, nil)                                                          \
  void integrate##N(BOOST_PP_ENUM_PARAMS(N, const ObjectsConstPtr &msg_in)) {                      \
    integrate({BOOST_PP_ENUM_PARAMS(N, msg_in)});                                                  \
  }
  // expands to definition of integrate2(), ..., integrate9(),
  // which just calls integrate() defined below
  BOOST_PP_REPEAT_FROM_TO(2, 10, ODM_DEFINE_INTEGRATE_N, nil)
#undef ODM_DEFINE_INTEGRATE_N

  void integrate(const std::vector<ObjectsConstPtr> &msgs_in) {
    const ObjectsPtr msg_out(new Objects());
    // header
    const ObjectsConstPtr latest_msg_in =
        *boost::max_element(msgs_in, [](const ObjectsConstPtr &a, const ObjectsConstPtr &b) {
          return a->header.stamp < b->header.stamp;
        });
    msg_out->header = latest_msg_in->header;
    // other fields
    for (const ObjectsConstPtr &msg_in : msgs_in) {
      // check frame id consistency?
      boost::copy(msg_in->names, std::back_inserter(msg_out->names));
      boost::copy(msg_in->probabilities, std::back_inserter(msg_out->probabilities));
      boost::copy(msg_in->contours, std::back_inserter(msg_out->contours));
    }
    pub_.publish(msg_out);
  }

private:
  boost::ptr_vector<message_filters::Subscriber<Objects>> subs_;
  std::shared_ptr<void> sync_;
  ros::Publisher pub_;
};

} // namespace object_detection_msgs

#endif