#!/usr/bin/env python
# BEGIN ALL
import rospy
import cv2
import cv_bridge
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan

past_ang = 0
angle_step_deg = 20


class Follower:
    perr = 0
    ptime = 0
    serr = 0
    dt = 0
    move = False
    past_ang = 0
    angle_step_deg = 20

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber(
            "/usb_cam/image_raw", Image, self.image_callback
        )
        self.lidar_sub = rospy.Subscriber("/scan_raw", LaserScan, self.lidar_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.image_pub = rospy.Publisher("/lane_image", Image, queue_size=1)

        self.twist = Twist()

        self.ray_angle = [x for x in range(angle_step_deg, 180, angle_step_deg)]
        self.dists = None

        self.cmd_vel_pub.publish(self.twist)

    def lidar_callback(self, msg):
        # get lidar distance at ray_angle in degree
        # dynamic offset
        # angles = [(x - 90) % 360 for x in self.ray_angle]
        # self.dists = [msg.ranges[x*2] for x in angles]
        # self.dists = list(map(lambda x: 0.1 if x == float('inf') else x, self.dists))
        # self.dists = list(map(lambda x: 0.5 if x >= 0.5 else x, self.dists))

        # static offset
        angles = [x for x in range(-10, -90, -5)]
        self.dists = [msg.ranges[x * 2] for x in angles]

    def get_obstacle_threshold(self):
        if self.dists is None:
            return 0

        # dynamic offset
        # lateral_dists = [dist * numpy.cos(numpy.deg2rad(theta)) for dist, theta in zip(self.dists, self.ray_angle)]

        # static offset
        lateral_count = 0
        for d in self.dists:
            if d < 0.5:
                lateral_count += 1
        if lateral_count >= 4:
            print("lateral_cnt :{}".format(lateral_count))
            return 10
        else:
            return 0

    # dynamic offset
    # return sum(lateral_dists)

    def image_callback(self, msg):
        img_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # original image backup for Tr
        img_color = cv2.resize(
            img_color, dsize=(320, 240), interpolation=cv2.INTER_AREA
        )
        imgb, imgg, imgr = cv2.split(img_color)

        # get size of image
        height, width = img_color.shape[:2]

        # perspective transform
        pts1 = np.float32([(123, 355), (491, 359), (30, 456), (624, 448)])
        pts1 = pts1 / 2
        pts2 = np.float32(
            [[5, 5], [width - 5, 5], [5, height - 5], [width - 5, height - 5]]
        )

        M = cv2.getPerspectiveTransform(pts1, pts2)
        img_result = cv2.warpPerspective(imgr, M, (width, height))

        # debugging
        # cv2.imshow("dst", img_result)

        roi_img = img_result

        roi_img[0 : int(height / 4), 0 : int(width)] = 0
        roi_img[int(height * 3 / 4) : height, 0 : int(width)] = 0

        # Opening
        roi_img = cv2.morphologyEx(
            roi_img, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        )
        # Canny Edge
        ret, roi_img = cv2.threshold(roi_img, 210, 255, cv2.THRESH_BINARY)

        # moment
        cxm = int(width / 2)
        M = cv2.moments(roi_img)
        if M["m00"] > 0:
            cxm = int(M["m10"] / M["m00"])

        dst = cv2.Canny(roi_img, 40, 200, None, 3)

        # get hough line
        linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 4, None, 22, 2)

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
                    slope = np.arctan(((l[1] - l[3]) / (l[2] - l[0]))) / np.pi * 180

                    if slope >= 5:
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

                    elif slope <= -5:
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

        if right_steer > 45:
            right_steer = 45.01
        if left_steer > 45:
            left_steer = -45

        if right_count > left_count:
            steer_angle = right_steer
        else:
            steer_angle = left_steer

        print(steer_angle)

        rad_steer_angle = steer_angle / 180 * np.pi

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
                int(width / 2 + 100 * (np.sin((rad_steer_angle)))),
                int(height / 2 - 100 * ((np.cos(rad_steer_angle)))),
            ),
            (0, 125, 255),
            5,
        )
        cv2.circle(
            dst,
            (
                int(width / 2 + 100 * (np.sin((rad_steer_angle)))),
                int(height / 2 - 100 * ((np.cos(rad_steer_angle)))),
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

        # debugging
        # cv2.imshow("dst", dst)
        # cv2.waitKey(1)

        if steer_angle == 0:
            steer_angle = past_ang
            print("No lane")
        else:
            self.past_ang = steer_angle
        cx = int(((width / 2 + steer_angle) + cxm - self.get_obstacle_threshold()) / 2)
        err = cx - (width / 2)
        K_p = 0.9
        self.dt = rospy.get_time() - self.ptime
        ang_z = (err / 100.0) * K_p + ((err - self.perr) / self.dt) / 20 / 100
        ang_z = min(0.6, max(-0.6, ang_z))

        lin_x = 0.8 * (1 - abs(ang_z))

        self.twist.linear.x = lin_x
        self.twist.angular.z = -ang_z
        self.perr = err
        self.ptime = rospy.get_time()

        self.cmd_vel_pub.publish(self.twist)
        output_img = self.bridge.cv2_to_imgmsg(dst)
        self.image_pub.publish(output_img)


rospy.init_node("follower")
follower = Follower()
rospy.spin()
# END ALL
