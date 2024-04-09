#!/usr/bin/env python3

import time

import rospy
from geometry_msgs.msg import PoseStamped
from utils import ndarray2pose_stamp_se2
from nav_msgs.msg import OccupancyGrid

from final_pnc.msg import ReachGoal

graph_search = [
    "a_star",
    "dijkstra",
    "d_star",
    "theta_star",
    "hybrid_a_star",
]
sample_based = [
    "rrt",
    "rrt_star",
    "informed_rrt_star",
    "prm",
    "rrt_connect",
    "rrt_star_connect",
]

class Evaluator:
    def __init__(self, target="start"):
        self.reach = False
        poses = rospy.get_param("me5413_world")
        self.global_planner = rospy.get_param("~global_planner", "a_star")
        self.local_planner = rospy.get_param("~local_planner", "dwa")
        self.goal = ndarray2pose_stamp_se2([poses[target]["x"], poses[target]["y"], poses[target]["yaw"]])
        self.goal.header.frame_id = "map"
        self.goal.header.stamp = rospy.Time.now()

        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.reach_sub = rospy.Subscriber("/final_pnc/reach_goal", ReachGoal, self.reach_callback)
        if self.global_planner in graph_search:
            self.expand_sub = rospy.Subscriber("/move_base/GraphPlanner/expand", OccupancyGrid, self.expand_callback)

    def reach_callback(self, msg: ReachGoal):
        self.reach = msg.result.data
    
    def expand_callback(self, msg: OccupancyGrid):
        pass

    def run(self):
        st = time.time()
        self.goal_pub.publish(self.goal)
        while not rospy.is_shutdown():
            if self.reach:
                break
            time.sleep(0.01)
        duration = time.time() - st


if __name__ == "__main__":
    rospy.init_node("pub_goal")
    eva = Evaluator(target="vehicle_2")
