#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class Pose2Odom:
    def __init__(self, pose_stamped_topic, odometry_topic):
        self.odometry_pub = rospy.Publisher(odometry_topic, Odometry, queue_size=10)
        self.pose_stamped_sub = rospy.Subscriber(pose_stamped_topic, PoseStamped, self.pose_2_odom_callback)

    def pose_2_odom_callback(self, pose_stamped_msg):
        odometry_msg = Odometry()
        odometry_msg.header = pose_stamped_msg.header
        odometry_msg.pose.pose = pose_stamped_msg.pose
        self.odometry_pub.publish(odometry_msg)

if __name__ == '__main__':
    rospy.init_node('pose2odom', anonymous=True)
    pose_stamped_topic = '/tracked_pose' 
    odometry_topic = '/final_slam/odom'
    repackage_node = Pose2Odom(pose_stamped_topic, odometry_topic)
    rospy.spin()