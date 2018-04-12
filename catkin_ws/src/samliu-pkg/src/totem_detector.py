#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import numpy as np
import copy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sys

class totem_detection_node():
	def __init__(self):
		self.img_sub = rospy.Subscriber("/rickbot/camera_node/image", Image, self.img_cb)
		self.img_pub = rospy.Publisher("totem_center",Image)
		self.bridge = CvBridge()
		self.cv_image = 0
		self.lock = 0
		

	def img_cb(self, data):
		#print "Image callback"
		if self.lock == 0:
			try:
				self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

			except CvBridgeError as e:
				print(e)
	
	def process(self):
		if type(self.cv_image) == np.int:
			return
		#print "process"
		self.lock = 1
		img = copy.copy(self.cv_image)
		self.lock = 0

		####### ADD YOUR CODE HERE #######
		img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		img = cv2.GaussianBlur(img, (19, 19), 0)

		hsvimg = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

		GreenHSVlow = np.array([45,110,80], dtype=np.uint8)
		GreenHSVhigh = np.array([60,200,160], dtype=np.uint8)
		green_threshed = cv2.inRange(hsvimg, GreenHSVlow, GreenHSVhigh)

		RedHSVlow = np.array([0,100,100], dtype=np.uint8)
		RedHSVhigh = np.array([20,255,255], dtype=np.uint8)
		red_threshed = cv2.inRange(hsvimg, RedHSVlow, RedHSVhigh)

		_,green_contours,green_hierarchy = cv2.findContours(green_threshed, 1, 2) # no erosion and no dilation
		_,red_contours,red_hierarchy = cv2.findContours(red_threshed, 1, 2)	 # no erosion and no dilation

		largest_green_area = 500
		largest_red_area = 500
		largest_green_index = 0
		largest_red_index = 0
		for index in range(len(green_contours)):
		    cnt = green_contours[index]
		    area = cv2.contourArea(cnt)
		    if area > largest_green_area:
		        #print index, area
		        largest_green_area = area
		        largest_green_index = index
		for index in range(len(red_contours)):
		    cnt = red_contours[index]
		    area = cv2.contourArea(cnt)
		    if area > largest_red_area:
		        #print index, area
		        largest_red_area = area 
		        largest_red_index = index
		#print largest_green_index, largest_red_index
		green_totem_contour = green_contours[largest_green_index]
		red_totem_contour = red_contours[largest_red_index]

		green_M = cv2.moments(green_totem_contour)
		red_M = cv2.moments(red_totem_contour)

		green_cx = int(green_M['m10']/green_M['m00'])
		green_cy = int(green_M['m01']/green_M['m00'])

		red_cx = int(red_M['m10']/red_M['m00'])
		red_cy = int(red_M['m01']/red_M['m00'])

		cv2.circle(img,(green_cx, green_cy),5,(0,255,0),2)
		cv2.circle(img,(red_cx, red_cy),5,(255,0,0),2)

		_ = cv2.circle(output_img,((green_cx+red_cx)/2, (green_cy+red_cy)/2),8,(0,255,255),-1)


		try:
			#self.img_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
			self.img_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
		except CvBridgeError as e:
			print(e)
		
def main(args):
	ic = totem_detection_node()
	rospy.init_node('totem_detection_node', anonymous = True)        
	try:
		while (1):
			ic.process()
			rospy.sleep(0.1)
	except KeyboardInterrupt:
		print("Shutting down")


if __name__ == '__main__':
	main(sys.argv)
