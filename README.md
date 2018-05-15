# object_detection_msgs
ROS message definitions for object detection in image

## Messages
[Objects](msg/Objects.msg)
* stamp, names, probabilities, and contours of detected objects

[Points](msg/Points.msg)
* contour points of a detected object

[Point](msg/Point.msg)
* an image coordinate

## Nodelet: SynchronizedObjectPublisher
* subscribe an image topic and publish user-specified objects synchronized to the image
* subscribe another object topic, merge it with user-specified objects and publish

### Subscribed topics
image_raw (sensor_msgs/Image)
* an image topic to be synchronized
* set ~subscribe_image true to subscribe

objects_in ([object_detection_msgs/Objects](msg/Objects.msg))
* an object topic to be merged and synchronized
* set ~subscribe_objects true to subscribe

### Published topics
objects_out ([object_detection_msgs/Objects](msg/Objects.msg))
* synchronized to an image or objects subscribed

### Services
get_objects ([object_detection_msgs/GetObjects](srv/GetObjects.srv))
* get user-specified objects

set_objects ([object_detection_msgs/SetObjects](srv/SetObjects.srv))
* set user-specified objects

### Parameters
~subscribe_image (bool, default: false)
* Either this or ~subscribe_objects must be true to let this nodelet work

~subscribe_objects (bool, default: false)
* Either this or ~subscribe_image must be true to let this nodelet work

~names (string[], default: \<empty array>)
* names of published objects

~probabilities (double[], default: \<empty array>)
* probabilities of published objects

~contours (int[][][2], default: \<empty array>)
* contours of published objects

~image_transport (string, default: "raw")
* transport type for the subscribed image topic

### Examples
see [launch/test.launch](launch/test.launch)