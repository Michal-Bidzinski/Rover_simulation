#!/usr/bin/python3.6
import rospy
import cv2
import numpy as np

# for LaserScan
#from sensor_msgs import laser_scan
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header


def main():

    # node init
    rospy.init_node('rover_laser_scan_Generator', anonymous = True)
    rate = rospy.Rate(30) # 30hz

    pub = rospy.Publisher('scan_generated', LaserScan, queue_size=2)

    scan = LaserScan()
    scan.header.frame_id = "map"
    scan.header.stamp = rospy.Time.now()
    scan.angle_min = -3.14
    scan.angle_max = 3.14
    scan.angle_increment = 6.28/200
    scan.time_increment = (1 / 30) / (200);
    scan.range_min = 0.0
    scan.range_max = 100.0
    #scan.set_ranges_size(100)
    #scan.set_intensities_size(100)
    
    for i in range(1, 200):
        print("i: ", i )
        scan.ranges.append(10+ i%10 * 0.1)
        scan.intensities.append(200) 
        

    print("create scan")
    # main loop 
    while not rospy.is_shutdown():
        # get and publish image and camera_info
        pub.publish(scan)
        rate.sleep()

if __name__ == '__main__':
    main()

