#!/usr/bin/python
import rospy
import cv2
import numpy as np
import argparse
from cv_bridge import CvBridge, CvBridgeError

# for point_cloud
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo, LaserScan, Image
from std_msgs.msg import Header


bridge = CvBridge()
def callback_depth(msg):
    print("get data")
    #msg.header.stamp = rospy.Time.now()
    #msg.header.frame_id = "camera"
    #pub_laser_scan.publish(msg)
    depth = bridge.imgmsg_to_cv2(msg)
    print("depth image type: ", type(depth))
    print("max: ", np.max(depth))
    print("max: ", np.min(depth))
    print("shape: ", depth.shape)
    
    depth_output = np.array(np.zeros([480,640]), dtype = np.dtype('f4'))

    thresh1_norm = cv2.normalize(thresh1,thresh1,0,1,cv2.NORM_MINMAX)

    depth_output =  thresh1_norm * self.depth
    

    cv2.imshow("image",depth_output)
    cv2.waitKey(0)


# Node init and publisher definition
rospy.init_node('depth_to_scan', anonymous = True)
pub_laser_scan = rospy.Publisher("laser_scan_from_depth", LaserScan, queue_size=2)
rospy.Subscriber("align_depth_sync", Image, callback_depth)
rate = rospy.Rate(30) # 30hz


print("Start node")

while not rospy.is_shutdown():

    rospy.spin()


