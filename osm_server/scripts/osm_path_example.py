#!/usr/bin/env python3.6
import sys
import rospy
from osm_server.srv import *
import re
from osm_server.msg import Array_my_a, Array_my_b
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion


class OSM_path_generator:

    def __init__(self):
        # read params

        # start and end point
        self.x_start = rospy.get_param("/start_lat")
        self.y_start = rospy.get_param("/start_lng")
        self.x_end   = rospy.get_param("/end_lat")
        self.y_end   = rospy.get_param("/end_lng")

        # bounding box or area
        self.lat1 = rospy.get_param("/lat1")
        self.lat2 = rospy.get_param("/lat2")
        self.lng1 = rospy.get_param("/lng1")
        self.lng2 = rospy.get_param("/lng2")

        # custom filter flag
        self.custom_filter = rospy.get_param("/aeroway_filter")

        # ros publisher and rate
        self.pub_waypoints_class = rospy.Publisher("waypoints", PoseArray, queue_size = 2) 
        self.rate_class = rospy.Rate(30)

        # tempolary variable
        self.result = None
        self.msg_array = PoseArray()

    # call osm service 
    def client_osm(self, arg):
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

        # show results
        for i, point in enumerate(waypoints_array):
            print('num: ', i, '-> ', point.position.x, '  ', point.position.y)
        self.msg_array.poses = waypoints_array
     
    # publish  waypoints
    def publish_waypoints(self):
        while not rospy.is_shutdown():
            self.pub_waypoints_class.publish(self.msg_array)
            self.rate_class.sleep()

def main():
    # ros init
    rospy.init_node('osm_path', anonymous = True) 

    # create object
    osm_object = OSM_path_generator()

    # call osm service
    osm_object.call_osm_client()

    # create message
    osm_object.create_msg_pose_array()

    # publish waypoints
    osm_object.publish_waypoints()

if __name__ == '__main__':
    main()
    
