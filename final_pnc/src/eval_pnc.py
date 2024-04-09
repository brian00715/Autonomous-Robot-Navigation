#!/usr/bin/env python3

import json
import os
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Imu
from utils import euclidian_dist_se2, ndarray2pose_stamp_se2, pose2ndarray_se2, quat2yaw
from final_pnc.msg import ReachGoal


def default_dump(obj):
    """Convert numpy classes to JSON serializable objects."""
    if isinstance(obj, (np.integer, np.floating, np.bool_)):
        return obj.item()
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    else:
        return obj


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

mid_point = [13, 3.25, 0]  # x,t,theta


class Evaluator:
    def __init__(self, target="start", global_planner="a_star", local_planner="dwa", param_dict=None, sample_rate=10):
        poses = rospy.get_param("me5413_world")
        self.global_planner = global_planner
        self.local_planner = local_planner
        self.goal_pose = ndarray2pose_stamp_se2([poses[target]["x"], poses[target]["y"], poses[target]["yaw"]])
        self.goal_pose.header.frame_id = "map"
        self.goal_pose.header.stamp = rospy.Time.now()
        self.param_dict = param_dict
        self.sample_rate = sample_rate  # hz

        # states
        self.reach = False
        self.curr_odom = None
        self.curr_pose_ar = None
        self.twist_ar = []
        self.acc_ar = []
        self.global_cell_num = []
        self.global_plan_time = []

        # ros related
        # pubs
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        if self.global_planner in graph_search:
            self.expand_sub = rospy.Subscriber("/move_base/GraphPlanner/expand", OccupancyGrid, self.expand_callback)
        # subs
        self.reach_sub = rospy.Subscriber("/final_pnc/reach_goal", ReachGoal, self.reach_callback)
        self.odom_sub = rospy.Subscriber("/gazebo/ground_truth/state", Odometry, self.odom_callback)
        self.imu_sub = rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        self.global_plan_time_sub = rospy.Subscriber(
            "/final_pnc/mpc/global_path_plan_time", Float32, self.global_plan_time_callback
        )
        # srvs
        # self.spawn_srv = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        rospy.loginfo("Evaluator initialized")

    def global_plan_time_callback(self, msg: Float32):
        self.global_plan_time.append(msg.data)

    def imu_callback(self, msg: Imu):
        self.acc_ar.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        pass

    def reach_callback(self, msg: ReachGoal):
        self.reach = msg.result.data
        pass

    def odom_callback(self, msg: Odometry):
        self.curr_odom = msg
        self.curr_pose_ar = pose2ndarray_se2(msg.pose.pose)
        self.twist_ar.append([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z])

    def expand_callback(self, msg: OccupancyGrid):
        grid_data = np.array(msg.data)
        count = np.count_nonzero(grid_data == 50)
        self.global_cell_num.append(count)

    def run(self):
        rate = rospy.Rate(self.sample_rate)
        for i in range(3):
            self.goal_pub.publish(self.goal_pose)
            rate.sleep()
        st = time.time()
        rospy.loginfo(
            f"Start evaluation with global planner: {self.global_planner}, local planner: {self.local_planner}"
        )
        # while not rospy.is_shutdown():
        #     self.goal_pub.publish(self.goal_pose)
        #     rate.sleep()
        while not rospy.is_shutdown():
            if self.reach:
                self.odom_sub.unregister()
                self.imu_sub.unregister()
                self.reach_sub.unregister()
                if self.global_planner in graph_search:
                    self.expand_sub.unregister()

                break
            rate.sleep()
        duration = time.time() - st
        self.acc_ar = np.array(self.acc_ar)
        self.twist_ar = np.array(self.twist_ar)

        acc_trans = np.linalg.norm(self.acc_ar[:, 0:2], axis=1)  # translation acceleration
        acc_rot = self.acc_ar[:, 2]
        vel_trans = np.linalg.norm(self.twist_ar[:, 0:2], axis=1)  # translation velocity
        vel_rot = self.twist_ar[:, 2]

        # evalutae navigation error
        pos_error = euclidian_dist_se2(self.curr_odom.pose.pose, self.goal_pose.pose)
        heading_error = abs(self.curr_pose_ar[2] - quat2yaw(self.goal_pose.pose.orientation))

        result_dict = {}
        result_dict["global_planner"] = {}
        result_dict["local_planner"] = {}
        result_dict["global_planner"]["method"] = self.global_planner
        result_dict["local_planner"]["method"] = self.local_planner
        result_dict["global_planner"]["visited_cells"] = int(np.sum(self.global_cell_num))
        if len(self.global_plan_time) == 0:
            self.global_plan_time = [-1]
        result_dict["global_planner"]["plan_time"] = float(np.sum(self.global_plan_time))
        result_dict["local_planner"]["param"] = self.param_dict["local_planner"]
        result_dict["global_planner"]["param"] = self.param_dict["global_planner"]
        goal_ar = pose2ndarray_se2(self.goal_pose.pose)
        result_dict["target"] = {"x": goal_ar[0], "y": goal_ar[1], "theta": goal_ar[2]}
        result_dict["duration"] = duration
        result_dict["pos_error"] = pos_error
        result_dict["heading_error"] = heading_error
        result_dict["success"] = "true" if self.reach else "false"

        date = time.strftime("%m-%d_%H-%M-%S", time.localtime())
        result_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), f"../result/{self.global_planner}_{self.local_planner}_{date}"
        )
        if not os.path.exists(result_dir):
            os.makedirs(result_dir)

        with open(os.path.join(result_dir, f"result.json"), "w") as f:
            f.write(json.dumps(result_dict, indent=4))

        np.save(f"{result_dir}/acc_trans.npy", acc_trans)
        np.save(f"{result_dir}/ac c_rot.npy", acc_rot)
        np.save(f"{result_dir}/vel_trans.npy", vel_trans)
        np.save(f"{result_dir}/vel_rot.npy", vel_rot)
        rospy.loginfo("Evaluation finished")


if __name__ == "__main__":
    rospy.init_node("eval_pnc")
    global_planner = rospy.get_param("~global_planner", "theta_star")
    local_planner = rospy.get_param("~local_planner", "mpc")

    # expereiment settings, used to write log
    param_dict = {
        "global_planner": {},
        "local_planner": {
            "max_vel_trans": 2,
            "max_vel_theta": 2,
            "max_lin_acc": 15,
            "max_ang_acc": 10,
            "xy_tol": 0.2,
            "yaw_tol": 5,  #  degree
        },
    }
    eva = Evaluator(
        target="vehicle_2", global_planner=global_planner, local_planner=local_planner, param_dict=param_dict
    )
    eva.run()
