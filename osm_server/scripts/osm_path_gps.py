#!/usr/bin/env python3.6
import sys
import rospy
from osm_server.srv import *
import re
from osm_server.msg import Array_my_a, Array_my_b
from sensor_msgs.msg import NavSatFix
from copy import copy 

from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion


class OSM_path_generator():
    def __init__(self):

        # start and end point
        self.x_start = 0.0
        self.y_start = 0.0
        self.x_end   = 0.0
        self.y_end   = 0.0

        # read params

        # bounding box or area
        self.lat1 = rospy.get_param("/lat1")
        self.lat2 = rospy.get_param("/lat2")
        self.lng1 = rospy.get_param("/lng1")
        self.lng2 = rospy.get_param("/lng2")

        # custom filter flag
        self.custom_filter = rospy.get_param("/aeroway_filter")

        # ros publisher, subscriber and rate
        self.pub_waypoints_class = rospy.Publisher("waypoints", PoseArray, queue_size = 2) 
        self.start_point_sub = rospy.Subscriber("gps_point_lat_lng", NavSatFix, self.gps_callback)
        self.end_point_sub = rospy.Subscriber("end_point_lat_lng", NavSatFix, self.end_callback)
        self.rate = rospy.Rate(30) 
      
        # tempolary variables
        self.publish = False
        self.msg_array = PoseArray()

    # gps module callback
    def gps_callback(self,data):

        self.x_start = data.latitude
        self.y_start = data.longitude
        print("gps_point: ", self.x_start, " ", self.y_start)

    # end point callback 
    def end_callback(self,data):

        self.x_end = data.latitude
        self.y_end = data.longitude
        print("end_point: ", self.x_end, " ", self.y_end)

        # prepare data to osm service and call service
        self.call_osm_client()

        # create message and show results
        self.create_msg_pose_array()
 

    # use osm servis for path generate 
    def client_osm(self,arg):
        rospy.wait_for_service('osm_path')
        try:
            generator = rospy.ServiceProxy('osm_path', OSM_path)
            resp = generator(arg)
            return resp.sum
        except rospy.ServiceException:
            print("Service call failed")

    # prepare data to osm service and call service
    def call_osm_client(self):
        arguments = [self.x_start, self.y_start, self.x_end, self.y_end, self.lat1, self.lat2, self.lng1, self.lng2, self.custom_filter]
        print(arguments)

        # call service
        self.result = self.client_osm(arguments)

    # create message
    def create_msg_pose_array(self):
        waypoints_array = [Pose(Point(l1.array2[0], l1.array2[1], 0.0), Quaternion(0.0, 0.0, 0.0, 0.0)) for l1 in self.result.array1]

        self.msg_array.poses = waypoints_array
        self.publish = True

        # show results
        for i, point in enumerate(waypoints_array):
            print('num: ', i, '-> ', point.position.x, '  ', point.position.y)

    # publish  waypoints
    def publish_waypoints(self):
        if self.publish:
            print("pub")
            self.pub_waypoints_class.publish(self.msg_array)


def main():
    
    # ros init
    rospy.init_node('osm_path', anonymous = True)  
    rate = rospy.Rate(30) 

    # create object
    osm_object = OSM_path_generator()

    # publish waypoints
    while not rospy.is_shutdown():
        osm_object.publish_waypoints()
        rate.sleep()

if __name__ == '__main__':
    main()
