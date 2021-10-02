#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import math
import enum
import warnings

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.image_pub = rospy.Publisher('/lane_image', Image, queue_size=1)
        self.twist = Twist()

        self.cmd_vel_pub.publish(self.twist)

    def image_callback(self, msg):

        #roi masking function
        def region_of_interest(img, vertices):
            mask = np.zeros_like(img)
            channel_count = img.shape[2]
            match_mask_color = (255,) * channel_count
            cv2.fillPoly(mask, vertices, match_mask_color)
            masked_image = cv2.bitwise_and(img, mask)
            return masked_image


        img_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        #original image backup for Tr
        img_original = img_color.copy()
        img_color = cv2.resize(img_color, dsize=(320, 240), interpolation=cv2.INTER_AREA)
	frame=img_color
        #get size of image
        height, width = img_color.shape[:2]

        class Color(enum.Enum):
            YELLO = 0
            GREEN = 1
            RED = 2

        warnings.filterwarnings("ignore")


        def find_circle(res, light_color):
            img = cv2.medianBlur(res, 5)
            ccimg = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
            cimg = cv2.cvtColor(ccimg, cv2.COLOR_BGR2GRAY)
            circles = cv2.HoughCircles(cimg, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=10,
                                       maxRadius=150)
            if circles is not None:
                #if light_color == Color.YELLO:
                  #  print('Yello Circle')
                if light_color == Color.GREEN:
                    print('Green Circle')
                if light_color == Color.RED:
                    print('RED Circle')

                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    cv2.circle(cimg, (i[0], i[1]), i[2], (0, 255, 0), 2)
                    cv2.circle(cimg, (i[0], i[1]), 2, (0, 0, 255), 3)
            return cimg

        
        hsv = cv2.cvtColor(img_color, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([20, 20, 100])
        upper_yellow = np.array([32, 255, 255])

        lower_green = np.array([50, 150, 50])
        upper_green = np.array([80, 255, 255])

        lower_red = np.array([-10, 100, 100])
        upper_red = np.array([30, 255, 255])

        yello_range = cv2.inRange(hsv, lower_yellow, upper_yellow)
        green_range = cv2.inRange(hsv, lower_green, upper_green)
        red_range = cv2.inRange(hsv, lower_red, upper_red)

        yello_res = cv2.bitwise_and(frame, frame, mask=yello_range)
        green_res = cv2.bitwise_and(frame, frame, mask=green_range)
        red_res = cv2.bitwise_and(frame, frame, mask=red_range)

	kernel=cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
	yello_res = cv2.morphologyEx(yello_res, cv2.MORPH_OPEN, kernel)
	green_res = cv2.morphologyEx(green_res, cv2.MORPH_OPEN, kernel)
	red_res = cv2.morphologyEx(red_res, cv2.MORPH_OPEN, kernel)

        yello_result = find_circle(yello_res, Color.YELLO)
        red_result = find_circle(red_res, Color.RED)
        green_result = find_circle(green_res, Color.GREEN)
 
        #cv2.imshow('yello circle', yello_result)
        cv2.imshow('red circle', red_result)
        cv2.imshow('green circle', green_result)

        cv2.waitKey(1)
	  


        self.cmd_vel_pub.publish(self.twist)
        output_img = self.bridge.cv2_to_imgmsg(img_color)
        self.image_pub.publish(output_img)

        # END CONTROL


rospy.init_node('traffic_light')
follower = Follower()
rospy.spin()
# END ALL

