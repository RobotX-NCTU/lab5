#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import numpy as np
import copy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sys
import time

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
		img_raw = copy.copy(self.cv_image)
		self.lock = 0

		####### ADD YOUR CODE HERE #######
		#start_time = time.time() # start time counting

		img = cv2.cvtColor(img_raw, cv2.COLOR_BGR2RGB)
		#img = cv2.GaussianBlur(img, (19, 19), 0)
		img = cv2.blur(img,(10,10))

		hsvimg = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

		GreenHSVlow = np.array([45,110,80], dtype=np.uint8)
		GreenHSVhigh = np.array([60,200,160], dtype=np.uint8)
		green_threshed = cv2.inRange(hsvimg, GreenHSVlow, GreenHSVhigh)

		RedHSVlow = np.array([0,100,100], dtype=np.uint8)
		RedHSVhigh = np.array([20,255,255], dtype=np.uint8)
		red_threshed = cv2.inRange(hsvimg, RedHSVlow, RedHSVhigh)

		_,green_contours,green_hierarchy = cv2.findContours(green_threshed, 1, 2) # no erosion and no dilation
		_,red_contours,red_hierarchy = cv2.findContours(red_threshed, 1, 2)	 # no erosion and no dilation

		# define a threshold for find the largest area
		largest_area_threshold = 20

		largest_green_area = 0
		largest_red_area = 0
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

		if largest_red_area > largest_area_threshold and largest_green_area > largest_area_threshold:
			green_totem_contour = green_contours[largest_green_index]
			red_totem_contour = red_contours[largest_red_index]

			green_M = cv2.moments(green_totem_contour)
			red_M = cv2.moments(red_totem_contour)

			# find centroid of totem
			#if green_M['m00'] != 0 and red_M['m00'] !=0:
			green_cx = int(green_M['m10']/green_M['m00'])
			green_cy = int(green_M['m01']/green_M['m00'])

			red_cx = int(red_M['m10']/red_M['m00'])
			red_cy = int(red_M['m01']/red_M['m00'])


			cv2.circle(img_raw,(green_cx, green_cy),5,(0,255,0),2)
			cv2.circle(img_raw,(red_cx, red_cy),5,(255,0,0),2)

			_ = cv2.circle(img_raw,((green_cx+red_cx)/2, (green_cy+red_cy)/2),8,(0,255,255),-1)
		else:
			print("no totem pair")
		
		try:
			#self.img_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
			self.img_pub.publish(self.bridge.cv2_to_imgmsg(img_raw, "bgr8"))
		except CvBridgeError as e:
			print(e)

		#print("--- %s seconds ---" % (time.time() - start_time))
		
def main(args):
	ic = totem_detection_node()
	rospy.init_node('totem_detection_node', anonymous = True)        
	
	while (1):
		try:
			ic.process()
			rospy.sleep(0.1)
		except KeyboardInterrupt:
			print("Shutting down")
			break



if __name__ == '__main__':
	main(sys.argv)
