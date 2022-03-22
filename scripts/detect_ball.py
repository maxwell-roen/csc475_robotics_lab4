#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge # for converting ROS img message to OpenCV img
from sensor_msgs.msg import Image

# first arg is (height, width, color channels (r, g, b))
rgb_img = np.zeros((720, 1280, 3), dtype='uint8')

# a flag to track that we have received our first image
img_received = False

def get_image(ros_img):
	global rgb_img
	global img_received
	# convert the ros img to an opencv img
	rgb_img = CvBridge().imgmsg_to_cv2(ros_img, 'rgb8')
	img_received = True
	# cv2.imwrite('capture.jpeg', rgb_img)

# do the actual img processing
def detect_ball(rgb_img):
	# first convert to HSV
	hsv = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2HSV)
	# define color ranges for the ball. not completely happy with these: having a hard time picking up
	# the black (logo) and white (lines) parts of the ball w/o introducing noise.
	lower_yellow_hsv = np.array([28, 8, 98])
	upper_yellow_hsv = np.array([65, 255, 255])
	# filter the img
	yellow_mask = cv2.inRange(hsv, lower_yellow_hsv, upper_yellow_hsv)
	
	# use a bitwise and mask to declare an area of interest, filtering out noise along the edges.
	mask = np.zeros((rgb_img.shape[0], rgb_img.shape[1], 1), dtype='uint8')
	area_of_interest = cv2.rectangle(mask, (200, 150), (1152, 648), 255, -1)

	# bitwise and the mask with the img
	filtered_img = cv2.bitwise_and(yellow_mask, area_of_interest, mask=area_of_interest)
	
	return filtered_img


if __name__ == '__main__':
	# define the node
	rospy.init_node('detect_ball', anonymous=True)
	# define a subscriber to read images
	img_sub = rospy.Subscriber('/camera/color/image_raw', Image, get_image)
	# define a publisher to publish the modified img
	img_pub = rospy.Publisher('/ball_2D', Image, queue_size = 1)
	
	# set loop frequency
	rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():
		# use the flag to make sure we don't try to start before receiving img
		if img_received:
			# do the img processing
			mono_img = detect_ball(rgb_img)
			# convert back to ros msg for publishing
			img_msg = CvBridge().cv2_to_imgmsg(mono_img, encoding='mono8')
			# publish it
			img_pub.publish(img_msg)
		# pause til next loop
		rate.sleep()
