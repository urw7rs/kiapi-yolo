#!/usr/bin/env python
# BEGIN ALL
import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cilab_line_follower.msg import pos

global perr, ptime, serr, dt, move
perr = 0
ptime = 0
serr = 0
dt = 0
move = False

perspective_points=np.zeros([4,2],dtype=np.float32)
idx=0


class Follower:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
		self.line_pub = rospy.Publisher('/line_position', pos, queue_size=1)
		self.image_pub = rospy.Publisher('/lane_image', Image, queue_size=1)
		self.position = pos()

		self.line_pub.publish(self.position)

	

	def image_callback(self, msg):
		global perr, ptime, serr, dt

		def mouse_callback(event, x, y, flags, param):
			global point_list_idx, idx

			if event == cv2.EVENT_LBUTTONDOWN:
				perspective_points[idx] = (x, y)
				idx = idx + 1

				print("(%d,%d)" % (x, y))

				cv2.circle(img_color, (x, y), 10, (0, 0, 255), -1)

		cv2.namedWindow('original')
		cv2.setMouseCallback('original', mouse_callback)

		image0 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		img_color = cv2.resize(image0, None, fx=0.6, fy=0.6, interpolation=cv2.INTER_CUBIC)
		
		img_original=img_color.copy()

		while (True):
			cv2.imshow('original', img_color)
			height, width = img_color.shape[:2]

			if cv2.waitKey(1) == 32:
				break
		dst = np.float32([[0, 0], [width, 0], [0, height], [width, height]])
		M = cv2.getPerspectiveTransform(perspective_points, dst)
		out_img = cv2.warpPerspective(img_original, M, (width, height))
		cv2.imshow('result', out_img)
		cv2.waitKey(0)
		
		self.line_pub.publish(self.position)
		output_img = self.bridge.cv2_to_imgmsg(out_img)
		self.image_pub.publish(output_img)

rospy.init_node('line_perspective_points', anonymous=True)
follower = Follower()
rospy.spin()

