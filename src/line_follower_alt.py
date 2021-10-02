#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import numpy as np


global perr, ptime, serr, dt, move
perr = 0
ptime = 0
serr = 0
dt = 0
move = False


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
        global perr, ptime, serr, dt
        img_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        img_original = img_color.copy()
        img_color = cv2.resize(
            img_color, dsize=(320, 240), interpolation=cv2.INTER_AREA
        )

        # perspective transform
        height, width, c = img_color.shape
        pts1 = np.float32([(123, 355), (491, 359), (30, 456), (624, 448)])
        pts2 = np.float32(
            [[10, 10], [width - 10, 10], [10, height - 10], [width - 10, height - 10]]
        )

        M = cv2.getPerspectiveTransform(pts1, pts2)

        img_result = cv2.warpPerspective(img_color, M, (width, height))
        img_result = cv2.bilateralFilter(img_result, 9, 75, 75)

        roi_img = cv2.cvtColor(img_result, cv2.COLOR_BGR2GRAY)
        roi_img = cv2.equalizeHist(roi_img)
        roi_img = cv2.morphologyEx(
            roi_img, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        )

        dst = cv2.Canny(roi_img, 150, 300, None, 3)
        dst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)

        # cv2.imshow("original", img_color)
        # cv2.imshow('color',img_result)
        # cv2.imshow('canny',dst)

        def getHistogram(img, minPer=0.1, display=False, region=1):

            if region == 1:
                histValues = np.sum(img, axis=0)  # axis=0 standard total
            else:
                histValues = np.sum(img[img_original.shape[0] // region :, :], axis=0)

            print("histValues:", histValues)
            maxValue = np.max(histValues)
            minValue = minPer * maxValue  # minimum percentatage
            # print('minValue:',minValue)

            indexArray = np.where(histValues >= minValue)
            basePoint = int(np.average(indexArray))
            # print('basepoint :',basePoint)  # middile point

            if display:
                imgHist = np.zeros(
                    (img_color.shape[0], img_color.shape[1], 3), np.uint8
                )
                for x, intensity in enumerate(histValues):
                    divided = img_color.shape[0] - intensity // 255 // region
                    cv2.line(
                        imgHist,
                        (x, img_color.shape[0]),
                        (x, divided[0]),
                        (255, 0, 255),
                        1,
                    )
                    cv2.circle(
                        imgHist,
                        (basePoint, img_color.shape[0]),
                        20,
                        (0, 255, 255),
                        cv2.FILLED,
                    )
                    # print('intensity:',intensity[0])
                    # print('divided:',divided[0])
                return basePoint, imgHist
            return basePoint

        basePoint, imgHist = getHistogram(
            img_result, display=True, minPer=0.5, region=4
        )
        curveAveragePoint, imgHist = getHistogram(img_result, display=True, minPer=0.9)
        ang_z = basePoint - curveAveragePoint

        self.twist.linear.x = 0.5
        self.twist.angular.z = ang_z
        # curveRaw = curveAveragePoint-basePoint

        # curveList = []
        # avgValue = 10
        # curveList.append(curveRaw)
        # if len(curveList)> avgValue:    # make less noise
        #     curveList.pop(0)
        # curve = int(sum(curveList/len(curveList)))

        # # inverse step is skipped

        # cv2.imshow("Histo", imgHist)

        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        self.cmd_vel_pub.publish(self.twist)

        # END CONTROL

        # cv2.imshow("win3", rgb_yw2)
        # cv2.waitKey(3)


rospy.init_node("follower")
follower = Follower()
rospy.spin()
# END ALL
