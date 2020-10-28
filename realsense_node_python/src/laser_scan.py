#!/usr/bin/python3.6
import pyrealsense2 as rs
import rospy
import cv2
import numpy as np
import argparse

# for point_cloud
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo, LaserScan
from std_msgs.msg import Header

def callback_scan(msg):
    print("get data")
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "camera"
    pub_laser_scan.publish(msg)

# Node init and publisher definition
rospy.init_node('laser_scan2', anonymous = True)
pub_laser_scan = rospy.Publisher("scan3", LaserScan, queue_size=2)
rospy.Subscriber("scan2", LaserScan, callback_scan)
rate = rospy.Rate(30) # 30hz


print("Start node")

while not rospy.is_shutdown():

    rospy.spin()


