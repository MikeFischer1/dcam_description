#!/usr/bin/env python
import roslib; roslib.load_manifest('dcam_description')
import sys
import rospy
import cv2
import cv2.cv
import os
import numpy as np
import time
from std_msgs.msg import String,Int32,Float32MultiArray,Int16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class depth_finder:
	def __init__(self):
		self.xyz_array = Float32MultiArray()
		self.xyz_array.data = []

		self.dT = 1

		#rospy.Timer(rospy.Duration(self.dT), self.loop, oneshot=False)


		self.x = 0.0
		self.y = 0.0
		self.z = 0.0

		self.r_v = 0.0

		self.pix_y = 0
		self.pix_z = 0

		self.xyz_pub = rospy.Publisher('button_xyz',Float32MultiArray)
		self.bridge = CvBridge()
        ## Bring in point cloud from message PointCloud2
		self.image_sub = rospy.Subscriber("/depth_camera_ir/depth_camera/depth/image_raw",Image,self.callback)
		self.pix_array = rospy.Subscriber("/yz_pix",Int16MultiArray,self.pixel_callback)

	def callback(self,data):
        ## Convert depth image to cv2 image
		
		depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")   
		print depth_image.shape
		

		depth_array = np.array(depth_image, dtype=np.float32)

		print depth_array.shape

		
		self.x = depth_array[self.pix_z,self.pix_y]

		self.r_v = self.x*np.sin(0.523599)
		self.y = (300-float(self.pix_y))*self.r_v/300
		self.z = (300-float(self.pix_z))*self.r_v/300

		self.xyz_array.data = []
		self.xyz_array.data.append(self.x)
		self.xyz_array.data.append(self.y)
		self.xyz_array.data.append(self.z)
		self.xyz_pub.publish(self.xyz_array)
		
		print self.x
		print self.y
		print self.z

	def pixel_callback(self,data):
		
		self.pix_y = data.data[0]
		self.pix_z = data.data[1]
	
	#def loop(self,event):

		#self.xyz_array.data = []
		#self.xyz_array.data.append(self.x)
		#self.xyz_array.data.append(self.y)
		#self.xyz_array.data.append(self.z)
		#self.xyz_pub.publish(self.xyz_array)

		


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