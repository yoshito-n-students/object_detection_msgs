std_msgs/Header header
string[] names                          # names of detected objects
float64[] probabilities                 # [0,1]. out of range means no probability info
object_detection_msgs/Points[] contours # pixel positions of detected object's contour
