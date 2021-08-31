#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3


class JoyTeleop:
    def __init__(self):
        self.x_speed_scale = rospy.get_param("~x_speed_scale")
        self.w_speed_scale = rospy.get_param("~w_speed_scale")
        self.velocity = Twist()
        self.rate = rospy.Rate(20)

        self.joy_data = None
        self.z = 0.0

        self.cmdVelPublisher = rospy.Publisher("/cmd_vel", Twist, queue_size=2)
        self.joySubscriber = rospy.Subscriber("joy", Joy, self.buttonCallback)
        self.loop()

    def buttonCallback(self, joy_data):
        self.joy_data = joy_data
        if joy_data.buttons[4] == 1:
            self.velocity.linear.x = self.x_speed_scale * joy_data.axes[4]
        else:
            self.velocity.linear = Vector3(0.0, 0.0, 0.0)

        if joy_data.buttons[5] == 1:
            self.velocity.angular = Vector3(0.0, 0.0, 0.0)
            self.z = 0.0

        self.cmdVelPublisher.publish(self.velocity)

    def loop(self):
        while not rospy.is_shutdown():
            if self.joy_data is not None:
                if self.z > 0.7:
                    self.z = 0.7
                elif self.z < -0.7:
                    self.z = -0.7
                else:
                    self.z += self.w_speed_scale * self.joy_data.axes[0]
            self.velocity.angular.z = self.z
            self.cmdVelPublisher.publish(self.velocity)
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("joy_teleop")
    joy = JoyTeleop()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("exception")
