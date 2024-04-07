#!/usr/bin/env python3
'''
 # @ Author: Kuankuan Sima
 # @ Email: smkk00715@gmail.com
 # @ Create Time: 2024-04-06 23:19:59
 # @ Modified time: 2024-04-07 14:00:05
 # @ Description:
 '''


import math
import os
import sys

import ipdb
import numpy as np
import rospy
import scipy.linalg
import tf.transformations as tft
import yaml
from dynamic_reconfigure.server import Server as DynServer
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from nmpc_controller import NMPCC
from std_msgs.msg import Bool
from utils import (
    euclidian_dist_se2,
    extract_interpolated_path,
    ndarray2pose_se2,
    path2ndarray_se2,
    quat2yaw,
    pose2ndarray_se2,
)


class PIDController:
    def __init__(self, kp=1, ki=0, kd=0, max_output=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.max_output = 100

    def get_output(self, curr, ref, dt):
        error = ref - curr
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        output = max(min(output, self.max_output), -self.max_output)
        return output


class NMPCNode:
    def __init__(self, method="mpc", freq=10):
        rospy.init_node("path_tracker_node")

        self.flag = True
        self.method = method

        self.enable_plan = False
        config_path = os.path.join(sys.path[0], "../config/nav_params/mpc.yaml")
        param_dict = yaml.safe_load(open(config_path, "r"))
        self.ref_path_topic = param_dict["ref_path_topic"]
        self.odom_topic = param_dict["odom_topic"]
        self.arrive_th = param_dict["arrive_th"]
        self.vel_ref = param_dict["vel_ref"]
        self.rot_th = np.deg2rad(param_dict["rot_th"])
        self.freq = param_dict["freq"]

        self.yaw_pid = PIDController(kp=1, ki=0, kd=0.1)

        if self.method == "mpc":
            self.N = param_dict["N"]
            self.T = param_dict["T"]
            prob_params = {
                "control_dim": 2,
                "state_dim": 3,
                "max_vel": param_dict["max_vel"],
                "max_lin_acc": param_dict["max_lin_acc"],
                "max_ang_acc": param_dict["max_ang_acc"],
                "max_omega": param_dict["max_omega"],
                "init_pose": np.array([0, 0, 0]),
            }
            self.Q = np.diag(param_dict["Q"])
            self.R = np.diag(param_dict["R"])
            self.controller = NMPCC(
                T=self.T,
                N=self.N,
                prob_params=prob_params,
                Q=self.Q,
                R=self.R,
            )

        # LQR parameters
        if self.method == "lqr":
            self.Q = np.diag([1, 1, 1])
            self.R = np.diag([0.1, 0.1])
            self.dt = 0.1

        self.curr_odom = Odometry()
        self.ref_path = None
        self.actual_ref_path = None # interpolated path from the original reference path given the MPC parameters
        self.curr_pose = None
        self.goal_pose = PoseStamped()

        # ROS related
        # pubs
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pred_pose_pub = rospy.Publisher("/final_pnc/mpc/pred_pose", PoseArray, queue_size=1)
        self.actual_ref_path_pub = rospy.Publisher("/final_pnc/mpc/actual_ref_path", Path, queue_size=1)
        # subs
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.robot_odom_callback)
        self.ref_path_sub = rospy.Subscriber(self.ref_path_topic, Path, self.ref_path_callback)
        self.goal_pose_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_pose_callback)
        self.reach_pub = rospy.Publisher("/final_pnc/reach_goal", Bool, queue_size=1)
        # self.dyn_client = DynServer(path_publisherConfig, self.dyn_callback)

    def goal_pose_callback(self, msg: PoseStamped):
        self.enable_plan = True
        self.goal_pose = msg
        rospy.loginfo(f"New goal {(msg.pose.position.x,msg.pose.position.y,quat2yaw(msg.pose.orientation))} received!")

    def ref_path_callback(self, msg: Path):
        self.ref_path = msg
        if self.enable_plan:
            self.actual_ref_path = extract_interpolated_path(msg, self.T, self.N + 1, self.vel_ref)

    def find_nearest_point(self):
        if self.ref_path is None or self.curr_pose is None:
            return None

        min_dist = float("inf")
        selected_idx = None
        robot_x, robot_y = self.curr_pose[0], self.curr_pose[1]
        robot_heading = self.curr_pose[2]

        for i, pose in enumerate(self.ref_path.poses):
            point_x, point_y = pose.pose.position.x, pose.pose.position.y
            base2point_yaw = math.atan2(point_y - robot_y, point_x - robot_x)
            dist = math.sqrt((point_x - robot_x) ** 2 + (point_y - robot_y) ** 2)

            # Calculate the absolute angle difference between robot heading and base2point_yaw
            angle_diff = abs(robot_heading - base2point_yaw)
            angle_diff = min(angle_diff, 2 * math.pi - angle_diff)  # Normalize angle to be within [0, π]

            # Threshold for angle difference can be adjusted. Here it's set to π/4 radians (45 degrees)
            if dist < min_dist and angle_diff <= math.pi / 4:
                min_dist = dist
                selected_idx = i

        return selected_idx

    def output_tum_format(self, odom, filename="trajectory.txt"):
        """
        Output the trajectory tracking data in TUM format given an Odometry message.
        """
        # Extract the timestamp
        timestamp = odom.header.stamp.to_sec()

        # Extract position data
        position = odom.pose.pose.position
        x, y, z = position.x, position.y, position.z

        # Extract orientation data in TUM format
        qx, qy, qz, qw = (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        )

        # Format the data in TUM format
        tum_data = f"{timestamp} {x:.4f} {y:.4f} {z:.4f} {qx:.4f} {qy:.4f} {qz:.4f} {qw:.4f}\r\n"

        # For this example, we print the data, but you can choose to write it to a file or handle it differently
        print(tum_data)
        filename = "/home/simon/ME5413_Planning_Project/trajectory.txt"
        with open(filename, "a") as file:
            file.write(tum_data)

    def dyn_callback(self, config, level):
        self.vel_ref = config["speed_target"]
        rospy.loginfo("Reconfigure Request: speed_target={}".format(self.vel_ref))
        return config

    def pos_to_state(self, pos):
        yaw = quat2yaw(pos.orientation)
        state = np.array([pos.position.x, pos.position.y, yaw])
        return state

    def robot_odom_callback(self, msg: Odometry):
        if self.flag:
            self.flag = False
            pose = msg.pose.pose
            self.init_pose = self.pos_to_state(pose)
            return
        self.world_frame = msg.header.frame_id
        self.robot_frame = msg.child_frame_id
        self.curr_odom = msg
        self.curr_pose = [
            self.curr_odom.pose.pose.position.x,
            self.curr_odom.pose.pose.position.y,
            quat2yaw(self.curr_odom.pose.pose.orientation),
        ]

        # self.output_tum_format(odom)

    def local_path_callback(self, msg: Path):
        self.actual_ref_path = [
            [
                msg.poses[i].pose.position.x,
                msg.poses[i].pose.position.y,
                quat2yaw(msg.poses[i].pose.orientation),
            ]
            for i in range(11, min(len(msg.poses), 11 + self.N + 1))
        ]
        # if len(self.local_ref_path) < self.N + 1:  # extend the goal_poses to N+1
        #     self.local_ref_path += [self.local_ref_path[-1]] * (self.N + 1 - len(self.local_ref_path))

        if self.curr_pose is not None:
            for i in range(len(self.actual_ref_path)):
                # if self.local_ref_path[i][2] < -math.pi:
                #     self.local_ref_path[i][2] += math.pi
                # if self.local_ref_path[i][2] > math.pi:
                #     self.local_ref_path[i][2] -= math.pi
                diff = self.actual_ref_path[i][2] - self.curr_pose[2]
                while diff > math.pi:
                    diff -= 2 * math.pi
                    diff += 2 * math.pi
                self.actual_ref_path[i][2] = self.curr_pose[2] + diff

    def compute_cmd_vel_mpc(self):
        if self.actual_ref_path is None:
            return

        mpc_ref_path = path2ndarray_se2(self.actual_ref_path)
        u = self.controller.solve(self.curr_pose, mpc_ref_path, [[self.vel_ref, 0]] * self.N)
        x_pred = self.controller.x_opti
        pose_array = PoseArray()
        pose_array.header.frame_id = self.world_frame
        for i in range(self.N + 1):
            pose = Pose()
            pose.position.x = x_pred[i][0]
            pose.position.y = x_pred[i][1]
            quat = tft.quaternion_from_euler(0, 0, x_pred[i][2])
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            pose_array.poses.append(pose)
        self.pred_pose_pub.publish(pose_array)

        cmd_vel = Twist()
        cmd_vel.linear.x = u[0][0]
        cmd_vel.angular.z = u[0][1]
        return cmd_vel

    def compute_lqr_gain(self, A, B):
        P = np.matrix(scipy.linalg.solve_continuous_are(A, B, self.Q, self.R))
        K = np.matrix(scipy.linalg.inv(self.R) * (B.T * P))
        return K

    def compute_cmd_vel_lqr(self):
        if self.actual_ref_path is None or len(self.actual_ref_path) == 0:
            return Twist()

        ref_pose = self.actual_ref_path[0]
        x_ref, y_ref, yaw_ref = ref_pose[0], ref_pose[1], ref_pose[2]
        x, y, yaw = self.curr_pose[0], self.curr_pose[1], self.curr_pose[2]

        x_error = x_ref - x
        y_error = y_ref - y
        yaw_error = yaw_ref - yaw

        A = np.matrix([[0, 0, -self.vel_ref * math.sin(yaw)], [0, 0, self.vel_ref * math.cos(yaw)], [0, 0, 0]])
        B = np.matrix([[math.cos(yaw), 0], [math.sin(yaw), 0], [0, 1]])

        K = self.compute_lqr_gain(A, B)

        x = np.matrix([x_error, y_error, yaw_error]).T
        u = -K * x

        cmd_vel = Twist()
        cmd_vel.linear.x = self.vel_ref + u[0, 0]
        cmd_vel.angular.z = u[1, 0]

        return cmd_vel

    def run(self):
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            if self.actual_ref_path is not None and self.curr_pose is not None and self.enable_plan:
                first_point = pose2ndarray_se2(self.actual_ref_path.poses[0].pose)
                if abs(self.curr_pose[2] - first_point[2]) > math.pi:
                    if self.curr_pose[2] > first_point[2]:
                        self.curr_pose[2] -= 2 * math.pi
                    else:
                        self.curr_pose[2] += 2 * math.pi
                if abs(self.curr_pose[2] - first_point[2]) > self.rot_th:
                    cmd_vel = Twist()
                    cmd_vel.angular.z = self.yaw_pid.get_output(self.curr_pose[2], first_point[2], 1 / self.freq)
                if self.method == "mpc":
                    cmd_vel = self.compute_cmd_vel_mpc()
                elif self.method == "lqr":
                    cmd_vel = self.compute_cmd_vel_lqr()
                if euclidian_dist_se2(ndarray2pose_se2(self.curr_pose), self.goal_pose.pose) < self.arrive_th:
                    self.enable_plan = False
                    cmd_vel = Twist()
                    self.pub_cmd_vel.publish(cmd_vel) # stop immediately
                    self.reach_pub.publish(Bool(True))
                    rospy.loginfo("Goal reached!")
                self.pub_cmd_vel.publish(cmd_vel)
                self.actual_ref_path_pub.publish(self.actual_ref_path)
            rate.sleep()


if __name__ == "__main__":
    try:
        path_tracker_node = NMPCNode(method="mpc")
        path_tracker_node.run()
    except rospy.ROSInterruptException:
        pass
