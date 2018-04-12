#!/usr/bin/env python

import numpy as np
import cv2
from matplotlib import pyplot as plt
import roslib
import rospy
import os
from sensor_msgs.msg import CompressedImage

image_pub = rospy.Publisher("/totem/compressed", CompressedImage, queue_size=10)

def callback(image_data):
	# Direct conversion to CV2
	np_arr = np.fromstring(image_data.data, np.uint8)
	img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
	height,width = img.shape[:2]
	red_buoy_contours = get_filtered_contours(img, "RED_BUOY")
	greed_buoy_contours = get_filtered_contours(img, "GREEN_BUOY")
	buoy_contours, GREEN_BUOY_MID, RED_BUOY_MID = [], [], []

	# check if the list is empty
	if red_buoy_contours:
		buoy_contours.extend(red_buoy_contours)
	if greed_buoy_contours:
		buoy_contours.extend(greed_buoy_contours)

	for (cnt, box, mean_color, contour_type) in buoy_contours:
		# plot box around contour
		x,y,w,h = box
		# find middle point of the buoy
		if contour_type == "RED_BUOY":
			RED_BUOY_MID = [x+w/2, y+h/2]
		if contour_type == "GREEN_BUOY":
			GREEN_BUOY_MID = [x+w/2, y+h/2]
		font = cv2.FONT_HERSHEY_SIMPLEX
		cv2.putText(img, contour_type, (x,y-5), font, 1, mean_color, 2)
		cv2.rectangle(img,(x,y),(x+w,y+h), mean_color, 8)

	# find the middle point of two buoy
	if not GREEN_BUOY_MID:
		middle_point = RED_BUOY_MID
	elif not RED_BUOY_MID:
		middle_point = GREEN_BUOY_MID
	else :
		middle_point = middle(GREEN_BUOY_MID, RED_BUOY_MID)

	#### Create CompressedImage ####
	msg = CompressedImage()
	msg.header.stamp = rospy.Time.now()
	msg.format = "jpeg"
	msg.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()
	# Publish new image
	image_pub.publish(msg)
	#os.system('rosrun image_transport republish compressed in:=/totem raw out:=/totem/image_raw')

	# control(middle_point, width)
	# img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
	# plt.imshow(img)
	# plt.show()


def get_filtered_contours(img, contour_type):
 
	#convert to hsv
	hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	if contour_type == "GREEN_BUOY":
		# mask of green
		GREEN = [np.array(x, np.uint8) for x in [[50,50,50], [70, 255, 255]] ]
		frame_threshed = cv2.inRange(hsv_img, GREEN[0], GREEN[1])

	elif contour_type == "RED_BUOY":
		# mask of red
		RED_LOWER = [np.array(x, np.uint8) for x in [[0,0,0], [10, 255, 255]] ]
		mask0 = cv2.inRange(hsv_img, RED_LOWER[0], RED_LOWER[1])
		RED_UPPER = [np.array(x, np.uint8) for x in [[160,0,0], [180, 255, 255]] ]
		mask1 = cv2.inRange(hsv_img, RED_UPPER[0], RED_UPPER[1])
		frame_threshed = mask0 + mask1

	# morphological opening
	kernel = np.ones((5,5), np.uint8)
	opening = cv2.morphologyEx(frame_threshed, cv2.MORPH_OPEN, kernel)

	# threshold
	ret,thresh = cv2.threshold(opening,36,255,0)

	filtered_contours = []

	im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
	contour_area = [ (cv2.contourArea(c), (c) ) for c in contours]
	contour_area = sorted(contour_area,reverse=True, key=lambda x: x[0])

	height,width = img.shape[:2]

	for (area,cnt) in contour_area:
		# plot box around contour
		x,y,w,h = cv2.boundingRect(cnt)
		box = (x,y,w,h)
		aspect_ratio = float(h)/w
		# d =  0.5*(x-width/2)**2 + (y-height)**2
		# if not(h>15 and w >10 and d < 120000):
		# 	continue
		# extra filtering to remove lines
		if not(h>25 and w>25):
			continue
		# if d>90000:
		# 	if not(h>35 and w>35):
		# 		continue
		if cv2.contourArea(cnt)==0:
			continue
		# val = cv2.arcLength(cnt,True)**2/ cv2.contourArea(cnt)
		# if val > 35: continue
		# rect = cv2.minAreaRect(cnt)
		# ctr, sides, deg = rect
		# val  = 0.5*cv2.arcLength(cnt,True) / (w**2+h**2)**0.5
		# if val < 1.12: continue
		if not(area > 1000): 
			continue
		if not(aspect_ratio > 2.0):
			continue

		mask = np.zeros(thresh.shape,np.uint8)
		cv2.drawContours(mask,[cnt],0,255,-1)
		mean_val = cv2.mean(img,mask = mask)
		filtered_contours.append( (cnt, box, mean_val, contour_type) )
    
	return filtered_contours


def middle(point1, point2):

	middle_point = []
	middle_point.append((point1[0] + point2[0])/2)
	middle_point.append((point1[1] + point2[1])/2)

	return middle_point


def control(mid, width):

	true_mid = width / 2
	TOPIC="cmd_course"

	if (mid[0] <= true_mid+50) and (mid[0] >= true_mid-50):
		os.system('rosrun robotx_tutorial pub_course.sh 0.2 0') # go straight
	elif (mid[0] > true_mid+50):
		os.system('rosrun robotx_tutorial pub_course.sh 0 -0.5') # right turn
	elif (mid[0] < true_mid-50):
		os.system('rosrun robotx_tutorial pub_course.sh 0 0.5') # left turn

	return 0


if __name__ == '__main__':
	# Initialize ros node
	rospy.init_node("image_reader")

	# Ros subscriber
	image_sub = rospy.Subscriber("/rrbot/camera1/image_raw/compressed", CompressedImage, callback, queue_size = 1)
	

	rospy.spin()