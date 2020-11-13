#!/usr/bin/python3.6
import pyrealsense2 as rs
import rospy
import cv2
import numpy as np
import open3d as o3d
import message_filters
import math as m

# for trajectory 
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from nav_msgs.msg import Path
from trajectory_fun import get_path_position_orientation
from nav_msgs.msg import Odometry

# for point_cloud
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, Image, PointField
from std_msgs.msg import Header

from sensor_msgs.msg import NavSatFix



def cameras_callback(odom_msg, image_msg, gps_msg):
    timestamp_pc = image_msg.header.stamp
    timestamp_tr = odom_msg.header.stamp
    timestamp_gps = gps_msg.header.stamp
    print("IM: ", timestamp_pc, " TR: ", timestamp_tr, " GPS: ", timestamp_gps)
    print("IM: ", float(str(timestamp_pc))/1000000000, " TR: ", float(str(timestamp_tr))/1000000000, " GPS: ", float(str(timestamp_gps))/1000000000)
    print("difference_1: ", timestamp_tr - timestamp_pc)
    print("difference_1: ", timestamp_tr - timestamp_gps)
    print("difference_1: ", timestamp_pc - timestamp_gps)

    image_msg.header.stamp = rospy.Time()   
    odom_msg.header.stamp = rospy.Time()  
    gps_msg.header.stamp = rospy.Time() 
 
    pub_odom.publish(odom_msg)
    pub_align_depth.publish(image_msg)
    pub_gps.publish(gps_msg)

# Node init 
rospy.init_node('sync', anonymous = True)

# Subscriber definition
odometry = message_filters.Subscriber('odom_t265', Odometry)
point_cloud = message_filters.Subscriber('align_depth', Image)
gps_data = message_filters.Subscriber('gps_point_lat_lng', NavSatFix)


ts = message_filters.ApproximateTimeSynchronizer([odometry, point_cloud, gps_data], 3, 0.8, allow_headerless=True)
ts.registerCallback(cameras_callback)


# Publisher definition
pub_odom = rospy.Publisher('odom_t265_sync2', Odometry, queue_size=20)
pub_align_depth = rospy.Publisher("align_depth_sync2", Image, queue_size=2)
pub_gps = rospy.Publisher('gps_point_lat_lng_sync2', NavSatFix, queue_size=2)
rate = rospy.Rate(30) # 30hz

#print("Start node")
rospy.loginfo("Sync server is run")


while not rospy.is_shutdown():
    
    # Get data from cameras
    rate.sleep()

