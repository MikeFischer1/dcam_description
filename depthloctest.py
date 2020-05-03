#!/usr/bin/env python
import roslib; roslib.load_manifest('dcam_description')
import sys
import rospy
import cv2
import cv2.cv
import os
import numpy as np
from std_msgs.msg import String,Int32,Float32MultiArray
from sensor_msgs.msg import Image,PointCloud2
from cv_bridge import CvBridge, CvBridgeError



class depth_finder:
	def __init__(self):
		self.image_pub = rospy.Publisher("depth_array",Float32MultiArray)
		self.bridge = CvBridge()
        ## Bring in point cloud from message PointCloud2
		self.image_sub = rospy.Subscriber("/depth_camera_ir/depth_camera/depth/image_raw",Image,self.callback)

	def callback(self,data):
        ## Convert depth image to cv2 image
		
		depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")   
		print depth_image.shape
		

		depth_array = np.array(depth_image, dtype=np.float32)
		
		print depth_array.shape
		print depth_array[400,400]

		#this_folder = os.path.dirname(__file__)
		#data_file = os.path.join(this_folder,'depth_proc.txt')
		#np.set_printoptions(threshold=sys.maxsize)
		#with open(data_file, 'w') as f:
		#	np.set_printoptions(threshold=np.inf)
		#	f.write('{}'.format(depth_array))


def main(args):
    df = depth_finder()
    rospy.init_node('depth_finder', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)