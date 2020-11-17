#!/usr/bin/python3.6
import rospy
import serial
import re
import numpy as np
from copy import copy
from utm import from_latlon
import math as m
import matplotlib.pyplot as plt

from sensor_msgs.msg import NavSatFix
from move_base_msgs.msg import MoveBaseActionResult
from move_base_action_servis import simple_move


class Sequence:
    def __init__(self):
        self.points_array = [[4.5, -3.0, 0.1],
                             [5.0,  7.0, 0.1],
                             [-4.8, 9.3, 0.1],
                             [-8.1, 3.9, 0.1],
                             [-3.3, -6.0, 0.1],
                             [0.0,  0.0, 0.1]]

    def do_sequence(self) -> None:
        for point in self.points_array:
            simple_move(point)


def main():
    # init ros node
    rospy.init_node('master', anonymous=True)
    # rate = rospy.Rate(1)

    sequence = Sequence()
    sequence.do_sequence()


if __name__ == '__main__':
    main()
