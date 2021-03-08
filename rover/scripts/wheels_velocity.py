#!/usr/bin/python3.6
import rospy
import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32


class wheels_velocity():
    def __init__(self):
        print("Start node")
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.right_vel = 0
        self.left_vel = 0
        self.wheelSeparation = 1.0
        self.ticksPerMeter = 10.0
        self.maxMotorSpeed = 30.0

    # publisher definition
    def set_publisher_subscriber(self):
        self.pub_right_wheel = rospy.Publisher("wheel_right_vel", Int32, queue_size=2)
        self.pub_left_wheel = rospy.Publisher("wheel_left_vel", Int32, queue_size=2)
        rospy.Subscriber("/husky_velocity_controller/cmd_vel", Twist, self.cmd_vel_callback)

    def cmd_vel_callback(self, data):
        #print(data)
        self.linear_vel = data.linear.x
        self.angular_vel = data.angular.z
        print("cmd_vel: ", self.linear_vel, self.angular_vel)
        velocity = self.getVelocity()
        self.left_vel = velocity[0]
        self.right_vel = velocity[1]
        self.publish_data()

    def getVelocity(self):
        tickRate = self.linear_vel*self.ticksPerMeter
        diffTicks = self.angular_vel*self.wheelSeparation*self.ticksPerMeter

        speeds = [0, 0]
        speeds[0] = tickRate - diffTicks
        speeds[1] = tickRate + diffTicks

        # Adjust speeds if they exceed the maximum.
        if max(speeds[0], speeds[1]) > self.maxMotorSpeed:
            factor = self.maxMotorSpeed / max(speeds[0], speeds[1])
            speeds[0] *= factor
            speeds[1] *= factor

        speeds[0] = int(speeds[0])
        speeds[1] = int(speeds[1])
        return speeds

    # publish messages
    def publish_data(self):
        # Publish right wheels velocity
        self.pub_right_wheel.publish(int(self.right_vel))

        # Publish left wheels velocity
        self.pub_left_wheel.publish(int(self.left_vel))


def main():
    # node init
    rospy.init_node('wheels_velocity', anonymous = True)
    rate = rospy.Rate(30) # 30hz

    velocity = wheels_velocity()
    velocity.set_publisher_subscriber()

    # main loop
    while not rospy.is_shutdown():
        # get and publish data
        rate.sleep()

if __name__ == '__main__':
    main()
