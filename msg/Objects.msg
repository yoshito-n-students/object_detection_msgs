std_msgs/Header header
string[] names                          # names of detected objects. 
                                        # empty string or no element mean no data.
float64[] probabilities                 # [0,1]. out of range or no element mean no probability info.
object_detection_msgs/Points[] contours # pixel positions of detected object's contour.
                                        # no points or no element mean no data.
