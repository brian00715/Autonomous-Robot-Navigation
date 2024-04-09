#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tf

class Odom2Pose:
    def __init__(self, pose_stamped_topic, odometry_topic):
        self.odometry_pub = rospy.Publisher(odometry_topic, Odometry, queue_size=10)
        self.pose_stamped_sub = rospy.Subscriber(pose_stamped_topic, PoseStamped, self.odom_2_pose_callback)

    def odom_2_pose_callback(self, pose_stamped_msg):
        odometry_msg = Odometry()
        odometry_msg.header = pose_stamped_msg.header
        odometry_msg.pose.pose = pose_stamped_msg.pose
        self.odometry_pub.publish(odometry_msg)

if __name__ == '__main__':
    rospy.init_node('odom2pose', anonymous=True)
    pose_stamped_topic = '/tracked_pose' 
    odometry_topic = '/final_slam/odom'
    repackage_node1= Odom2Pose(pose_stamped_topic, odometry_topic)
    rospy.spin()