#!/usr/bin/env python
import roslib; roslib.load_manifest('dcam_description')
import sys
import rospy
import cv2
import cv2.cv
import message_filters
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class depth_finder:
        def __init__(self):
        self.bridge = CvBridge()

        depth_sub = message_filters.Subscriber("camera/depth/image", Image)

        self.body_cascade = cv2.CascadeClassifier('/usr/local/share/OpenCV/haarcascades/haarcascade_upperbody.xml')
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def callback(self, rgb_data, depth_data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "passthrough")
        except CvBridgeError, e:
            print e

        depth_array = np.array(depth_image, dtype=np.float32)
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)

        print depth_array(1 ,2)

def main(args):
    fp = depth_finder()
    rospy.init_node('depth_finder', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)