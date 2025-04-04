#!/usr/bin/env python3

"""get the ground truth odometry from the gazebo simulation"""

import numpy as np
import rospy
import tf
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


class GetGtOdom:
    def __init__(self, model_name=[], odom_pub_rate=10, pub_tf=False):
        if model_name == []:
            raise ValueError("model_name is empty")
        self.gt_odom = {}
        self.gt_odom_pubs = {}
        self.model_name = model_name
        self.odom_pub_rate = odom_pub_rate
        self.pub_tf = pub_tf
        for i in range(len(model_name)):
            self.gt_odom[model_name[i]] = Odometry()
            self.gt_odom_pubs[model_name[i]] = rospy.Publisher(
                "/" + model_name[i] + "/gt_odom", Odometry, queue_size=10
            )
        if self.pub_tf:
            self.tf_broadcaster = tf.TransformBroadcaster()
        self.model_states_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)

    def model_states_callback(self, msg: ModelStates):
        for i in range(len(self.model_name)):
            idx = msg.name.index(self.model_name[i])
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "map"
            self.gt_odom[self.model_name[i]].header = header
            self.gt_odom[self.model_name[i]].child_frame_id = msg.name[idx]
            self.gt_odom[self.model_name[i]].pose.pose = msg.pose[idx]
            self.gt_odom[self.model_name[i]].twist.twist = msg.twist[idx]
            self.gt_odom_pubs[self.model_name[i]].publish(self.gt_odom[self.model_name[i]])

    def run(self):
        rate = rospy.Rate(self.odom_pub_rate)
        while not rospy.is_shutdown():
            rate.sleep()
            for i in range(len(self.model_name)):
                self.gt_odom_pubs[self.model_name[i]].publish(self.gt_odom[self.model_name[i]])
                if self.pub_tf:
                    self.tf_broadcaster.sendTransform(
                        (
                            self.gt_odom[self.model_name[i]].pose.pose.position.x,
                            self.gt_odom[self.model_name[i]].pose.pose.position.y,
                            self.gt_odom[self.model_name[i]].pose.pose.position.z,
                        ),
                        (
                            self.gt_odom[self.model_name[i]].pose.pose.orientation.x,
                            self.gt_odom[self.model_name[i]].pose.pose.orientation.y,
                            self.gt_odom[self.model_name[i]].pose.pose.orientation.z,
                            self.gt_odom[self.model_name[i]].pose.pose.orientation.w,
                        ),
                        rospy.Time.now(),
                        "/base_link",
                        "/odom",
                    )


if __name__ == "__main__":
    rospy.init_node("get_gt_odom", anonymous=True)
    get_gt_odom = GetGtOdom(model_name=["jackal"], odom_pub_rate=50, pub_tf=True)
    get_gt_odom.run()
