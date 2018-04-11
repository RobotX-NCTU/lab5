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
		self.img_pub = rospy.Publisher("totem_center",Image,queue_size=10)
		self.bridge = CvBridge()
		self.cv_image = 0
		self.lock = 0	
		

	def img_cb(self, data):
		if self.lock == 0:
			#print "Image callback"
			try:
				self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

			except CvBridgeError as e:
				print(e)
	
	def process(self):
		if type(self.cv_image) == np.int:
			return
		self.lock = 1
		img = self.cv_image
		#print image.shape
				
				img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		hsvimg = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
				GreenHSVlow = np.array([45,110,80], dtype=np.uint8)
				GreenHSVhigh = np.array([60,200,200], dtype=np.uint8)
				green_threshed = cv2.inRange(hsvimg, GreenHSVlow, GreenHSVhigh)
		RedHSVlow = np.array([0,100,100], dtype=np.uint8)
		RedHSVhigh = np.array([20,255,255], dtype=np.uint8)
		red_threshed = cv2.inRange(hsvimg, RedHSVlow, RedHSVhigh)

		kernel = np.ones((3,3),np.uint8)
		green_erosion = cv2.erode(green_threshed,kernel,iterations = 1)
		red_erosion = cv2.erode(red_threshed,kernel,iterations = 1)
		kernel = np.ones((5,5),np.uint8)
		green_dilation = cv2.dilate(green_erosion,kernel,iterations = 2)
		red_dilation = cv2.dilate(red_erosion,kernel,iterations = 2)

		_,green_contours,green_hierarchy = cv2.findContours(green_dilation, 1, 2)
		_,red_contours,red_hierarchy = cv2.findContours(red_dilation, 1, 2)

		if len(green_contours) == 0:
			try:
				self.img_pub.publish(self.bridge.cv2_to_imgmsg(output_img, "bgr8"))
			except CvBridgeError as e:
				print(e)
			return
		if len(red_contours) == 0:
			try:
				self.img_pub.publish(self.bridge.cv2_to_imgmsg(output_img, "bgr8"))
			except CvBridgeError as e:
				print(e)
			return
		gflag = 0
		rflag = 0
		largest_green_area = 2000
		largest_red_area = 2000
		largest_green_index = 0
		largest_red_index = 0
		for index in range(len(green_contours)):
			cnt = green_contours[index]
			area = cv2.contourArea(cnt)
			if area > largest_green_area:
				#print index, area
			gflag = 1
				largest_green_area = area
				largest_green_index = index
		for index in range(len(red_contours)):
			cnt = red_contours[index]
			area = cv2.contourArea(cnt)
			if area > largest_red_area:
				#print index, area
				rflag = 1
			largest_red_area = area 
				largest_red_index = index
		
		if gflag == 0 or rflag == 0:
			
			try:
				self.img_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
			except CvBridgeError as e:
				print(e)
			return
		#print largest_green_index, largest_red_index
		green_totem_contour = green_contours[largest_green_index]
		red_totem_contour = red_contours[largest_red_index]

		green_M = cv2.moments(green_totem_contour)
		red_M = cv2.moments(red_totem_contour)

		green_cx = int(green_M['m10']/green_M['m00'])
		green_cy = int(green_M['m01']/green_M['m00'])

		red_cx = int(red_M['m10']/red_M['m00'])
		red_cy = int(red_M['m01']/red_M['m00'])

		output_img = copy.copy(img)
		_ = cv2.circle(output_img,(green_cx, green_cy),5,(0,255,0),2)
		_ = cv2.circle(output_img,(red_cx, red_cy),5,(255,0,0),2)
		_ = cv2.circle(output_img,((green_cx+red_cx)/2, (green_cy+red_cy)/2),8,(0,255,255),-1)

		try:
			self.img_pub.publish(self.bridge.cv2_to_imgmsg(output_img, "bgr8"))
		except CvBridgeError as e:
			print(e)

		self.lock = 0
def main(args):
	ic = totem_detection_node()
	rospy.init_node('totem_detection_node', anonymous = True)        
	try:
		while (1):
			ic.process()
	except KeyboardInterrupt:
		print("Shutting down")


if __name__ == '__main__':
	main(sys.argv)
