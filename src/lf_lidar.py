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
    light_state = False  # red=False, green=True
    serr = 0

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

    def find_circle(self, res, light_color):
        img = cv2.medianBlur(res, 5)
        ccimg = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        cimg = cv2.cvtColor(ccimg, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(
            cimg,
            cv2.HOUGH_GRADIENT,
            1,
            20,
            param1=50,
            param2=1,
            minRadius=1,
            maxRadius=200,
        )
        if circles is not None:
            if light_color == 1:
                print("Green Circle Found")
                self.light_state = True
            if light_color == 2:
                print("RED Circle")  # Useless
            circles = np.uint16(np.around(circles))
        # return cimg

    def image_callback(self, msg):
        color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # original image backup for Tr
        color_img = cv2.resize(
            color_img, dsize=(self.width, self.height), interpolation=cv2.INTER_AREA
        )

        if self.light_state is False:  # Red Light
            hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
            sen = 10
            lower_green = np.array([48 - sen, 100, 100])
            upper_green = np.array([65 + sen, 255, 255])
            green_range = cv2.inRange(hsv, lower_green, upper_green)
            green_res = cv2.bitwise_and(color_img, color_img, mask=green_range)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            green_res = cv2.morphologyEx(green_res, cv2.MORPH_OPEN, kernel)
            green_res = cv2.dilate(green_res, kernel, iterations=5)
            self.find_circle(green_res, 1)
            # cv2.imshow('green', green_res)
            # cv2.waitKey(1)

        else:  # Green Light
            self.remove_yellow(color_img)
            _, _, img_red = cv2.split(color_img)
            img_red = cv2.morphologyEx(
                img_red,
                cv2.MORPH_OPEN,
                cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
            )
            img_red = cv2.warpPerspective(img_red, self.M, (self.width, self.height))
            img_red[: int(self.height / 4), : int(self.width)] = 0
            img_red[int(self.width * 3 / 4) : self.width, : int(self.width)] = 0

            image = cv2.warpPerspective(color_img, self.M, (300, 300))  # img_size
            ###
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            lower_red1 = np.array([0, 50, 50])
            upper_red1 = np.array([15, 255, 255])
            lower_red2 = np.array([160, 50, 50])
            upper_red2 = np.array([180, 255, 255])
            maskr1 = cv2.inRange(hsv, lower_red1, upper_red1)
            maskr2 = cv2.inRange(hsv, lower_red2, upper_red2)
            maskr = cv2.add(maskr1, maskr2)

            rgb_yb = cv2.bitwise_and(image, image, mask=maskr).astype(np.uint8)
            rgb_yb = cv2.cvtColor(rgb_yb, cv2.COLOR_RGB2GRAY)

            # filter mask
            kernel = np.ones((7, 7), np.uint8)
            opening = cv2.morphologyEx(rgb_yb, cv2.MORPH_OPEN, kernel)
            rgb_yb2 = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
            _, rgb_yb2 = cv2.threshold(rgb_yb2, 70, 255, cv2.THRESH_BINARY)

            # ROI
            out_img = rgb_yb2.copy()
            h, w = out_img.shape
            search_top = int(1 * h / 4 + 20)
            search_bot = int(3 * h / 4 + 20)
            search_mid = int(w / 2)
            out_img[:search_top, :w] = 0
            out_img[search_bot:h, :w] = 0

            M = cv2.moments(out_img)
            out_img = cv2.cvtColor(out_img, cv2.COLOR_GRAY2BGR)
            cxm = int(self.width / 2)
            if M["m00"] > 0:  # momentum
                cxm = int(M["m10"] / M["m00"])
                cym = int(M["m01"] / M["m00"])
                # cx = cxm - 110 #120#143 #CW
                # cx = cxm - 150
                offset = self.get_obstacle_threshold()
                # print("offset: ", offset)
                cx = cxm  # - offset

                cv2.circle(out_img, (cxm, cym), 20, (255, 0, 0), -1)
                cv2.circle(out_img, (cx, cym), 20, (255, 0, 0), 2)

                cx = cxm
                err = cx - (self.width / 2)
                self.dt = rospy.get_time() - self.ptime
                K_p = 0.0027
                K_d = 0.0005

                ang_z = (
                    err * K_p + (err - self.perr) / self.dt * K_d
                )  # + (self.serr*self.dt)*0.0002
                ang_z = min(0.8, max(-0.8, ang_z))
                lin_x = 1 * (1 - 0.5 * abs(ang_z))

                self.twist.linear.x = lin_x  # lin_x  # lin_x  # lin_x
                self.twist.angular.z = -ang_z
                self.perr = err
                self.ptime = rospy.get_time()
                self.serr = err + self.serr
                # cv2.imshow('out_img', out_img)
                # cv2.waitKey(1)

                self.cmd_vel_pub.publish(self.twist)
            else:
                _, img_red = cv2.threshold(img_red, 210, 255, cv2.THRESH_BINARY)

                dst = cv2.Canny(img_red, 40, 200, None, 3)

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
                            slope = (
                                np.arctan(((l[1] - l[3]) / (l[2] - l[0]))) / np.pi * 180
                            )

                            if slope >= 5:
                                cv2.line(
                                    out_img,
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
                                    out_img,
                                    (l[0], l[1]),
                                    (l[2], l[3]),
                                    (0, 255, 0),
                                    3,
                                    cv2.LINE_AA,
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
                rad_steer_angle = steer_angle / 180 * np.pi

                # draw steering angle
                cv2.line(
                    out_img,
                    (int(self.width / 2), int(self.height / 2)),
                    ((int(self.width / 2), int(self.height / 2) - 150)),
                    (244, 5, 52),
                    3,
                )
                cv2.line(
                    out_img,
                    (int(self.width / 2), int(self.height / 2)),
                    (
                        int(self.width / 2 + 100 * (np.sin((rad_steer_angle)))),
                        int(self.height / 2 - 100 * ((np.cos(rad_steer_angle)))),
                    ),
                    (0, 125, 255),
                    5,
                )
                cv2.circle(
                    out_img,
                    (
                        int(self.width / 2 + 100 * (np.sin((rad_steer_angle)))),
                        int(self.height / 2 - 100 * ((np.cos(rad_steer_angle)))),
                    ),
                    10,
                    (0, 125, 255),
                    -1,
                )
                cv2.putText(
                    out_img,
                    str(steer_angle),
                    (int(self.width / 2 + 10), int(self.height / 2 + 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                )

                if steer_angle == 0:
                    steer_angle = self.past_ang
                    print("No lane")
                else:
                    past_ang = steer_angle
                # cx = int((width / 2 + steer_angle)+cxm)/2  # - self.get_obstacle_threshold()

                # draw steering angle for debugging
                self.draw_direction(dst, steer_angle)

                if steer_angle == 0:
                    steer_angle = self.past_ang
                    print("No lane")
                else:
                    self.past_ang = steer_angle

                # cx = int((self.width / 2 + 7*steer_angle))
                cx = int(self.width / 2 + steer_angle)
                err = cx - (self.width / 2)
                self.dt = rospy.get_time() - self.ptime
                K_p = 1
                # K_d = 0.0006

                ang_z = err * K_p  # + (err - self.perr) / self.dt * K_d
                ang_z = min(0.8, max(-0.8, ang_z))
                lin_x = 0.8 * (1 - abs(ang_z))

                self.twist.linear.x = 0.2  # lin_x  # lin_x
                self.twist.angular.z = -ang_z
                self.perr = err
                self.ptime = rospy.get_time()
                # cv2.imshow('out_img', out_img)
                cv2.waitKey(1)

                self.cmd_vel_pub.publish(self.twist)


rospy.init_node("follower")
follower = Follower()
rospy.spin()
# END ALL
