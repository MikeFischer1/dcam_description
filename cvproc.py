#!/usr/bin/env python
import roslib
roslib.load_manifest('dcam_description')
import sys
import rospy
import cv2
from std_msgs.msg import String,Int16MultiArray
from sensor_msgs.msg import Image
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class image_converter:
	def __init__(self):
		self.pixel_array = Int16MultiArray()
		self.pixel_array.data = []
		## Output image with circle
		## self.image_pub = rospy.Publisher("cv_image",Image)
		
		self.pix_array = rospy.Publisher("yz_pix",Int16MultiArray)

		## CV Bridge the raw image in
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/depth_camera_ir/depth_camera/color/image_raw",Image,self.callback)

	def callback(self,data):
		## Convert raw image to CV-usable data, known as cv_image
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		gimg = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
		gimg = cv2.medianBlur(cv_image,5)
		#edge filter to get a black and white image
		canny = cv2.Canny(gimg, 200, 200)
		#detector must work only on canny (edge) image.
		circles = cv2.HoughCircles(canny,cv2.cv.CV_HOUGH_GRADIENT,1,20,
							param1=90,param2=20,minRadius=5,maxRadius=30)
		#print cv_image.shape
		circles = np.uint16(np.around(circles))
		for i in circles[0,:]:
		# draw the outer circle
			cv2.circle(cv_image,(i[0],i[1]),i[2],(0, 255, 0),2)
		# draw the center of the circle
			cv2.circle(cv_image,(i[0],i[1]),2,(0,0,255),3)

		##try:
		##	self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		##except CvBridgeError as e:
		##	print(e)

		#print i[0]
		#print i[1]
		self.pixel_array.data = []
		self.pixel_array.data.append(i[0])
		self.pixel_array.data.append(i[1])
		self.pix_array.publish(self.pixel_array)




def main(args):
	ic = image_converter()
	rospy.init_node('image_converter', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)