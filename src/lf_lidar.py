#!/usr/bin/env python
# BEGIN ALL
import rospy
import cv2
import cv_bridge
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan


class Follower:
    perr = 0
    ptime = 0
    dt = 0
    past_ang = 0
    lidar_count = 0

    def __init__(self, width=320, height=240, angle_step_deg=20):
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

        self.height = height
        self.width = width

        pts1 = np.float32([(61, 177), (209, 179), (15, 228), (312, 224)])
        pts2 = np.float32(
            [[5, 5], [width - 5, 5], [5, height - 5], [width - 5, height - 5]]
        )

        self.M = cv2.getPerspectiveTransform(pts1, pts2)

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
        # lateral_dists = [dist * numpy.cos(numpy.deg2rad(theta))
        # for dist, theta in zip(self.dists, self.ray_angle)]

        # static offset
        lateral_count = 0
        for d in self.dists:
            if d < 0.5:
                lateral_count += 1
        if lateral_count >= 4:
            print("lateral_cnt :{}".format(lateral_count))
            self.lidar_count += 1
            return 10
        else:
            if self.lidar_count:
                self.lidar_count -= 1
                return -10
            else:
                return 0

    # dynamic offset
    # return sum(lateral_dists)
    def remove_yellow(self, dst):
        hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)
        lower_yello = np.array([15, 30, 30])
        upper_yello = np.array([65, 255, 255])
        mask = cv2.inRange(hsv, lower_yello, upper_yello)
        dst[mask > 0] = (0, 0, 0)

    def draw_direction(self, dst, steer_angle):
        height, width = dst.shape[:2]

        rad_steer_angle = steer_angle / 180 * np.pi

        cv2.line(
            dst,
            (int(width / 2), int(height / 2)),
            (int(width / 2), int(height / 2 - 150)),
            (244, 5, 52),
            3,
        )
        cv2.line(
            dst,
            (int(width / 2), int(height / 2)),
            (
                int(width / 2 + 100 * np.sin(rad_steer_angle)),
                int(height / 2 - 100 * np.cos(rad_steer_angle)),
            ),
            (0, 125, 255),
            5,
        )
        cv2.circle(
            dst,
            (
                int(width / 2 + 100 * (np.sin((rad_steer_angle)))),
                int(height / 2 - 100 * np.cos(rad_steer_angle)),
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

        output_img = self.bridge.cv2_to_imgmsg(dst)
        self.image_pub.publish(output_img)

    def image_callback(self, msg):
        color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # original image backup for Tr
        color_img = cv2.resize(
            color_img, dsize=(self.width, self.height), interpolation=cv2.INTER_AREA
        )
        self.remove_yellow(color_img)

        blue, green, red = cv2.split(color_img)

        # perspective transform
        roi = cv2.warpPerspective(red, self.M, (self.width, self.height))

        # debugging
        # cv2.imshow("dst", img_result)

        roi[: int(self.height / 4), :] = 0
        roi[int(self.height * 3 / 4) :, :] = 0

        # Opening
        roi = cv2.morphologyEx(
            roi, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        )

        # Canny Edge

        dst = cv2.Canny(roi, 40, 200, None, 3)

        # get hough line
        linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 4, None, 22, 2)

        slope_pos = np.empty((0.0, 0.0))
        slope_neg = np.empty((0.0, 0.0))

        right_count = 0
        left_count = 0

        # transfrom gray to BGR
        dst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)

        if linesP:
            for i in range(len(linesP)):
                line = linesP[i][0]
                if line[2] - line[0]:
                    # get line's slope (degree)
                    slope = (
                        np.arctan(((line[1] - line[3]) / (line[2] - line[0])))
                        / np.pi
                        * 180
                    )

                    def draw_line(color):
                        cv2.line(
                            dst,
                            (line[0], line[1]),
                            (line[2], line[3]),
                            color,
                            3,
                            cv2.LINE_AA,
                        )

                    if slope >= 5:
                        draw_line((0, 255, 255))
                        slope_pos = np.append(slope_pos, slope)
                        right_count += 1

                    elif slope <= -5:
                        draw_line((0, 255, 0))
                        slope_neg = np.append(slope_neg, slope)
                        left_count += 1

        if slope_pos.size == 0:
            slope_pos = 90.0
        if slope_neg.size == 0:
            slope_neg = -90.0

        right_steer = 90 - np.mean(slope_pos)
        left_steer = -90 - np.mean(slope_neg)

        right_steer = min(right_steer, 45)
        left_steer = -min(left_steer, 45)

        if right_count > left_count:
            steer_angle = right_steer
        else:
            steer_angle = left_steer

        print(steer_angle)

        # draw steering angle for debugging
        self.draw_direction(dst, steer_angle)

        # debugging
        # cv2.imshow("dst", dst)
        # cv2.waitKey(1)

        if steer_angle == 0:
            steer_angle = self.past_ang
            print("No lane")
        else:
            self.past_ang = steer_angle

        # moment
        M = cv2.moments(roi)
        if M["m00"] > 0:
            cX = M["m10"] / M["m00"]
        else:
            cX = self.width / 2

        err = cX
        err += steer_angle
        err -= self.get_obstacle_threshold()
        err /= 2

        K_p = 0.009
        K_d = 0.2

        self.dt = rospy.get_time() - self.ptime

        ang_z = err * K_p + (err - self.perr) / self.dt * K_d

        self.twist.angular.z = -min(0.6, max(-0.6, ang_z))
        self.twist.linear.x = 0.8 * (1 - abs(ang_z))

        self.perr = err
        self.ptime = rospy.get_time()

        self.cmd_vel_pub.publish(self.twist)


rospy.init_node("follower")
follower = Follower()
rospy.spin()
# END ALL
