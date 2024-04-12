#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
import rospy
from utils import ndarray2pose_stamp_se2

# target = "start"
# target = "vehicle_2"

if __name__ == "__main__":
    rospy.init_node("pub_goal")
    target = rospy.get_param("target", "vehicle_2")
    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
    rate = rospy.Rate(1)

    poses = rospy.get_param("me5413_world")

    goal = ndarray2pose_stamp_se2([poses[target]["x"], poses[target]["y"], poses[target]["yaw"]])
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()

    while not rospy.is_shutdown():
        pub.publish(goal)
        # rate.sleep()
        break
