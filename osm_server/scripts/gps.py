#!/usr/bin/python3.6
import rospy
import serial
import re
from copy import copy

from sensor_msgs.msg import NavSatFix
from gps_library import gps_read_coordinates

class GPS():
    def __init__(self):
        # definition of serial port
        self.port="/dev/ttyUSB0"
        self.ser=serial.Serial(self.port, baudrate=9600, timeout=0.5)

        # receive coordinates flag
        self.received = False

        # coordinates
        self.lat = 0.0
        self.lng = 0.0

        # time
        self.time = None

        # previous coordinates
        self.lat_p = 0.0
        self.lng_p = 0.0

        # coordinates message
        self.msg = NavSatFix()

        # coordinates publisher
        self.gps_pub = rospy.Publisher("gps_point_lat_lng", NavSatFix, queue_size=2)

    def get_coordinates(self):
        while not rospy.is_shutdown():
            # read serial port
            newdata=self.ser.readline()

            # decode data
            self.received, self.lat, self.lng, self.time = gps_read_coordinates(newdata)

            # publish data if data is not identical (from this same frame)
            if self.received and abs(self.lat - self.lat_p) > 1.0e-17 and abs(self.lat - self.lat_p) > 1.0e-17:
                print("Lat: ", self.lat, " Lng: ", self.lng)

                # publish coordinates
                self.publish_waypoints()

                # update previous coordinates
                self.lat_p = copy(self.lat)
                self.lng_p = copy(self.lng)


    # publish  coordinates
    def publish_waypoints(self):
        self.msg.latitude = self.lat
        self.msg.longitude = self.lng
        self.msg.header.stamp = self.time
        self.gps_pub.publish(self.msg)


def main():
    # init ros node
    rospy.init_node('gps', anonymous = True)

    # create gps object
    gps_object = GPS()

    # read coordinates from gps module
    gps_object.get_coordinates()


if __name__ == '__main__':
    main()
