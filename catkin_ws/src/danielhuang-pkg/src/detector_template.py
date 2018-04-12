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
		try:
			self.img_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
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
