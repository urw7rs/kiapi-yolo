#!/usr/bin/env python
# BEGIN ALL
import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import math


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber(
            "/usb_cam/image_raw", Image, self.image_callback
        )
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.image_pub = rospy.Publisher("/lane_image", Image, queue_size=1)
        self.twist = Twist()

        self.cmd_vel_pub.publish(self.twist)

    def image_callback(self, msg):

        # roi masking function
        def region_of_interest(img, vertices):
            mask = np.zeros_like(img)
            channel_count = img.shape[2]
            match_mask_color = (255,) * channel_count
            cv2.fillPoly(mask, vertices, match_mask_color)
            masked_image = cv2.bitwise_and(img, mask)
            return masked_image

        img_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # original image backup for Tr
        img_original = img_color.copy()
        img_color = cv2.resize(
            img_color, dsize=(320, 240), interpolation=cv2.INTER_AREA
        )
        imgb, imgg, img_color = cv2.split(img_color)

        # get size of image
        height, width = img_color.shape[:2]

        # perspective transform
        pts1 = np.float32([(123, 355), (491, 359), (30, 456), (624, 448)])
        pts2 = np.float32(
            [[10, 10], [width - 10, 10], [10, height - 10], [width - 10, height - 10]]
        )

        cv2.circle(img_color, (int(pts1[0][0]), int(pts1[0][1])), 10, (255, 0, 0), -1)
        cv2.circle(img_color, (int(pts1[1][0]), int(pts1[1][1])), 10, (255, 255, 0), -1)
        cv2.circle(img_color, (int(pts1[2][0]), int(pts1[2][1])), 10, (255, 0, 255), -1)
        cv2.circle(img_color, (int(pts1[3][0]), int(pts1[3][1])), 10, (0, 0, 255), -1)

        M = cv2.getPerspectiveTransform(pts1, pts2)
        img_result = cv2.warpPerspective(img_original, M, (width, height))

        # debug
        # Perspective Transform result image
        # cv2.imshow("perspective Transform result", img_result)

        #####
        # region_of_interest_vertices = [(0, height/2), (0, height), (width, height), (width, height / 2)]
        # roi_img = region_of_interest(img_result, np.array([region_of_interest_vertices], np.int32))
        # cv2.imshow('ffff',roi_img)

        # Blur
        img_result = cv2.bilateralFilter(img_result, 9, 75, 75)

        roi_img = cv2.cvtColor(img_result, cv2.COLOR_BGR2GRAY)

        # Histogram equalizing
        roi_img = cv2.equalizeHist(roi_img)
        # Opening
        roi_img = cv2.morphologyEx(
            roi_img, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        )
        # Canny Edge
        dst = cv2.Canny(roi_img, 40, 100, None, 3)

        ##

        # get hough line
        linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 8, None, 30, 2)

        slope_pos = np.empty((0, 0), dtype=float)
        slope_neg = np.empty((0, 0), dtype=float)

        right_count = 0
        left_count = 0

        # transfrom gray to BGR
        dst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)

        if linesP is not None:
            for i in range(0, len(linesP)):
                l = linesP[i][0]
                if l[2] - l[0] != 0:
                    # get line's slope (degree)
                    slope = math.atan(((l[1] - l[3]) / (l[2] - l[0]))) / math.pi * 180

                    if slope >= 1:
                        cv2.line(
                            dst,
                            (l[0], l[1]),
                            (l[2], l[3]),
                            (0, 255, 255),
                            3,
                            cv2.LINE_AA,
                        )
                        slope_pos = np.append(slope_pos, slope)
                        right_count = right_count + 1

                    elif slope <= -1:
                        cv2.line(
                            dst, (l[0], l[1]), (l[2], l[3]), (0, 255, 0), 3, cv2.LINE_AA
                        )
                        slope_neg = np.append(slope_neg, slope)
                        left_count = left_count + 1

        if slope_pos.size == 0:
            slope_pos = 90.0
        if slope_neg.size == 0:
            slope_neg = -90.0

        right_steer = 90 - np.mean(slope_pos)
        left_steer = -90 - np.mean(slope_neg)

        if right_steer > 35:
            right_steer = 35.01
        if left_steer > 35:
            left_steer = -35

        if right_count > left_count:
            steer_angle = right_steer
        else:
            steer_angle = left_steer

        print(steer_angle)
        rad_steer_angle = steer_angle / 180 * math.pi

        height, width = dst.shape[:2]

        # draw steering angle
        cv2.line(
            dst,
            (int(width / 2), int(height / 2)),
            ((int(width / 2), int(height / 2) - 150)),
            (244, 5, 52),
            3,
        )
        cv2.line(
            dst,
            (int(width / 2), int(height / 2)),
            (
                int(width / 2 + 100 * (math.sin((rad_steer_angle)))),
                int(height / 2 - 100 * ((math.cos(rad_steer_angle)))),
            ),
            (0, 125, 255),
            5,
        )
        cv2.circle(
            dst,
            (
                int(width / 2 + 100 * (math.sin((rad_steer_angle)))),
                int(height / 2 - 100 * ((math.cos(rad_steer_angle)))),
            ),
            10,
            (0, 125, 255),
            -1,
        )
        cv2.putText(
            dst,
            str(steer_angle),
            (int(width / 2 + 10), int(height / 2 + 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
        )
        #####

        # result image(dst)
        # cv2.imshow("dst", dst)

        cv2.waitKey(1)

        rad_steer_angle = -rad_steer_angle

        self.twist.linear.x = 0.2
        self.twist.angular.z = rad_steer_angle

        self.cmd_vel_pub.publish(self.twist)
        output_img = self.bridge.cv2_to_imgmsg(dst)
        self.image_pub.publish(output_img)

        # END CONTROL


rospy.init_node("follower")
follower = Follower()
rospy.spin()
# END ALL
