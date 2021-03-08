#!/usr/bin/env python  
import roslib
import rospy
import tf

from nav_msgs.msg import Odometry


def callback(msg):

    br1 = tf.TransformBroadcaster()
    br1.sendTransform((- msg.pose.pose.position.y, 
                       - msg.pose.pose.position.x,
                       msg.pose.pose.position.z),
                     (0, 
                      0, 
                      0, 
                      1),
                     rospy.Time.now(),
                     "central",
                     "odom")

    br2 = tf.TransformBroadcaster()
    br2.sendTransform((0, 
                       0,
                       0),
                      (- msg.pose.pose.orientation.x, 
                       - msg.pose.pose.orientation.y, 
                       - msg.pose.pose.orientation.z, 
                       msg.pose.pose.orientation.w),
                      rospy.Time.now(),
                      "base_link",
                      "central")

    br3 = tf.TransformBroadcaster()
    br3.sendTransform((0.0, 
                       0.20,
                       0),
                      (0, 
                       0, 
                       1, 
                       0),
                      rospy.Time.now(),
                      "laser_link",
                      "base_link")
    # print("pub transform")




if __name__ == '__main__':
    rospy.init_node('tf_camera')
    rospy.Subscriber('odom_t265',
                     Odometry,
                     callback)
    
    rospy.loginfo("TF publisher is run")

    rospy.spin()
