#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import tf


class FixGPSOdom:
    def __init__(self, freq=60):
        self.odom_gps_sub = rospy.Subscriber("/odometry/gps", Odometry, self.fix_gps_odom_callback)
        self.fix_odom_gps_pub = rospy.Publisher("/odometry/gps/fix", Odometry, queue_size=10)
        self.gps_msg = None
        self.freq = freq
        self.tf_pub = tf.TransformBroadcaster()

    def fix_gps_odom_callback(self, msg: Odometry):
        self.gps_msg = msg

    def run(self):
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            new_msg = Odometry()
            if self.gps_msg is not None:
                new_msg = self.gps_msg
                new_msg.header.frame_id = "odom"
                new_msg.header.stamp = rospy.Time.now()
                new_msg.header.seq += 1
                new_msg.child_frame_id = "base_link"
                self.fix_odom_gps_pub.publish(self.gps_msg)
                self.tf_pub.sendTransform(
                    (new_msg.pose.pose.position.x, new_msg.pose.pose.position.y, new_msg.pose.pose.position.z),
                    (
                        new_msg.pose.pose.orientation.x,
                        new_msg.pose.pose.orientation.y,
                        new_msg.pose.pose.orientation.z,
                        new_msg.pose.pose.orientation.w,
                    ),
                    new_msg.header.stamp,
                    "base_link",
                    "odom",
                )
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("fix_gps_odom", anonymous=False)
    fix_gps_odom = FixGPSOdom()
    fix_gps_odom.run()
