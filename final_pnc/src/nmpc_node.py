#!/usr/bin/env python3
"""
 # @ Author: Kuankuan Sima
 # @ Email: smkk00715@gmail.com
 # @ Create Time: 2024-04-06 23:19:59
 # @ Modified time: 2024-04-07 14:00:05
 # @ Description:
 """

import time
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
from nav_msgs.srv import GetPlan
from nmpc_controller import NMPCC
from std_msgs.msg import Bool, Int8, Float32
from utils import (
    euclidian_dist_se2,
    find_nearest_point,
    get_acute_angle,
    ndarray2pose_se2,
    path2ndarray_se2,
    path_lin_interpo_cut,
    pose2ndarray_se2,
    quat2yaw,
    reorder_path_points,
    find_point_from_idx_dist,
    path_lin_interpo,
    NavStatus,
)

from final_pnc.msg import ReachGoal


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

        self.method = method

        # Load parameters
        config_path = os.path.join(sys.path[0], "../config/nav_params/mpc.yaml")
        param_dict = yaml.safe_load(open(config_path, "r"))
        self.ref_path_topic = param_dict["ref_path_topic"]
        self.odom_topic = param_dict["odom_topic"]
        self.make_plan_topic = param_dict["make_plan_topic"]
        self.make_plan_local_topic = param_dict["make_plan_local_topic"]
        self.xy_tol = param_dict["xy_tol"]
        self.ang_tol = np.deg2rad(param_dict["ang_tol"])
        self.vel_ref = param_dict["vel_ref"]
        self.rot_th = np.deg2rad(param_dict["rot_th"])
        self.freq = param_dict["freq"]
        self.local_window_size = param_dict["local_window_size"]

        self.yaw_pid = PIDController(kp=2, ki=0, kd=0.1)

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

        # States variables
        self.enable_ctrl = False
        self.curr_odom = Odometry()
        self.ref_path = None
        self.interpo_ref_path = None  # interpolated path from the original reference path given the MPC parameters
        self.curr_pose = None
        self.goal_pose = None
        self.global_path = None
        self.rot2start_yaw = False
        self.rot2goal_yaw = False

        # ROS related
        # pubs
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pred_pose_pub = rospy.Publisher("/final_pnc/mpc/pred_pose", PoseArray, queue_size=1)
        self.interpo_ref_path_pub = rospy.Publisher("/final_pnc/mpc/interpo_ref_path", Path, queue_size=1)
        self.local_path_pub = rospy.Publisher("/final_pnc/mpc/local_path", Path, queue_size=1)
        self.global_path_plan_time_pub = rospy.Publisher("/final_pnc/mpc/global_path_plan_time", Float32, queue_size=1)
        self.win_interp_point_pub = rospy.Publisher("/final_pnc/mpc/win_interp_point", PoseStamped, queue_size=1)
        # subs
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.robot_odom_callback)
        # self.ref_path_sub = rospy.Subscriber(self.ref_path_topic, Path, self.ref_path_callback)
        self.goal_pose_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_pose_callback)
        self.reach_pub = rospy.Publisher("/final_pnc/reach_goal", ReachGoal, queue_size=1)
        self.status_pub = rospy.Publisher("/final_pnc/status", Int8, queue_size=1)
        self.set_vel_sub = rospy.Subscriber("/final_pnc/set_ref_vel", Twist, self.set_speed_callback)
        # srvs
        try:
            rospy.wait_for_service(self.make_plan_topic, timeout=5)
        except rospy.ROSException:
            rospy.logerr(f"Service {self.make_plan_topic} is not available")
        try:
            rospy.wait_for_service(self.make_plan_local_topic, timeout=5)
        except rospy.ROSException:
            rospy.logerr(f"Service {self.make_plan_local_topic} is not available")
        self.get_plan_srv = rospy.ServiceProxy(self.make_plan_topic, GetPlan)
        self.get_plan_local_srv = rospy.ServiceProxy(self.make_plan_local_topic, GetPlan)
        # self.dyn_client = DynServer(path_publisherConfig, self.dyn_callback)

        rospy.loginfo("Path tracker node initialized")

    def set_speed_callback(self, msg: Twist):
        self.vel_ref = msg.linear.x
        self.controller.set_param("max_vel", self.vel_ref)
        rospy.loginfo(f"New reference vel:{self.vel_ref} received!")

    def pub_error(self):
        status = Int8()
        status.data = NavStatus.FAILED
        self.status_pub.publish(status)
        self.enable_ctrl = False

    def goal_pose_callback(self, msg: PoseStamped):
        if self.curr_odom is None:
            return
        self.goal_pose = msg
        curr_pose = PoseStamped()
        curr_pose.header = self.curr_odom.header
        curr_pose.header.frame_id = "map"
        curr_pose.pose = self.curr_odom.pose.pose
        st = time.time()
        try:
            self.global_path = self.get_plan_srv.call(curr_pose, msg, 0).plan
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            self.pub_error()
            return
        if len(self.global_path.poses) == 0 or self.global_path is None:
            rospy.logerr("Global path planning failed!")
            self.pub_error()
            return
        duration = time.time() - st
        self.global_path_plan_time_pub.publish(duration)
        self.global_path = path_lin_interpo(self.global_path, 0.2)
        rospy.loginfo(f"New goal {(msg.pose.position.x,msg.pose.position.y,quat2yaw(msg.pose.orientation))} received!")
        self.rot2start_yaw = True
        self.enable_ctrl = True

    def ref_path_callback(self, msg: Path):
        self.ref_path = msg
        if self.enable_ctrl:
            self.interpo_ref_path = path_lin_interpo_cut(msg, self.T, self.N + 1, self.vel_ref)

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

    def robot_odom_callback(self, msg: Odometry):
        self.world_frame = msg.header.frame_id
        self.robot_frame = msg.child_frame_id
        self.curr_odom = msg
        self.curr_pose = [
            self.curr_odom.pose.pose.position.x,
            self.curr_odom.pose.pose.position.y,
            quat2yaw(self.curr_odom.pose.pose.orientation),
        ]
        # rospy.loginfo(f"Current pose: {self.curr_pose}")
        # self.output_tum_format(odom)

    def local_path_callback(self, msg: Path):
        self.interpo_ref_path = [
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
            for i in range(len(self.interpo_ref_path)):
                diff = self.interpo_ref_path[i][2] - self.curr_pose[2]
                while diff > math.pi:
                    diff -= 2 * math.pi
                while diff < -math.pi:
                    diff += 2 * math.pi
                self.interpo_ref_path[i][2] = self.curr_pose[2] + diff

    def compute_cmd_vel_mpc(self):
        if self.interpo_ref_path is None:
            return

        mpc_ref_path = path2ndarray_se2(self.interpo_ref_path)
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
        if self.interpo_ref_path is None or len(self.interpo_ref_path) == 0:
            return Twist()

        ref_pose = self.interpo_ref_path[0]
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

    def is_xy_reached(self):
        return euclidian_dist_se2(ndarray2pose_se2(self.curr_pose), self.goal_pose.pose) < self.xy_tol

    def run(self):
        rate = rospy.Rate(self.freq)
        cmd_vel = Twist()
        while not rospy.is_shutdown():

            if self.curr_odom is not None and self.enable_ctrl:
                status = Int8()
                status.data = NavStatus.EXECUTING
                self.status_pub.publish(status)

                if self.is_xy_reached():
                    rospy.loginfo_throttle(1, "XY reached!")
                    cmd_vel = Twist()
                    self.pub_cmd_vel.publish(cmd_vel)  # stop immediately
                    self.rot2start_yaw = False
                    self.rot2goal_yaw = True

                idx_odom_in_path = find_nearest_point(self.global_path, self.curr_pose)
                if 0:
                    if idx_odom_in_path is None:
                        rospy.logwarn_throttle(1, "can't find nearest point")
                        continue
                    self.ref_path = Path()
                    self.ref_path.header = self.global_path.header
                    self.ref_path.poses = self.global_path.poses[idx_odom_in_path:]
                if 1:
                    win_inter_point = find_point_from_idx_dist(
                        self.global_path, idx_odom_in_path, self.local_window_size
                    )
                    if win_inter_point is None:
                        rospy.logwarn_throttle(1, "can't find nearest point")
                        continue
                    self.win_interp_point_pub.publish(win_inter_point)
                    curr_pose_stam = PoseStamped()
                    curr_pose_stam.header = self.curr_odom.header
                    curr_pose_stam.header.frame_id = "map"
                    curr_pose_stam.pose = self.curr_odom.pose.pose
                    local_path = self.get_plan_local_srv.call(curr_pose_stam, win_inter_point, 0).plan
                    if local_path is None or len(local_path.poses) == 0:
                        local_path = self.get_plan_local_srv.call(
                            curr_pose_stam, self.goal_pose, 0
                        ).plan  # if fail, replan to goal
                        if local_path is None or len(local_path.poses) == 0:
                            rospy.logwarn_throttle(1, "local_path planning failed!")
                            status = Int8()
                            status.data = NavStatus.FAILED
                            self.status_pub.publish(status)
                            continue
                    self.local_path_pub.publish(local_path)
                    self.ref_path = local_path

                self.interpo_ref_path = path_lin_interpo_cut(self.ref_path, self.T, self.N + 1, self.vel_ref)
                self.interpo_ref_path.header = self.curr_odom.header
                self.interpo_ref_path_pub.publish(self.interpo_ref_path)

                if self.rot2start_yaw:
                    rospy.loginfo_throttle(1, "Rotating to start yaw")
                    ref_point = pose2ndarray_se2(self.interpo_ref_path.poses[2].pose)
                    ref_yaw = np.arctan2(ref_point[1] - self.curr_pose[1], ref_point[0] - self.curr_pose[0])
                    ref_yaw = get_acute_angle(self.curr_pose[2], ref_yaw)
                    # rospy.loginfo(f"Rotating: {self.curr_pose[2]:.2f} -> {ref_yaw:.2f}")
                    if abs(ref_yaw - self.curr_pose[2]) < self.rot_th:
                        self.rot2start_yaw = False
                        self.rot2goal_yaw = False
                        continue
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0
                    cmd_vel.angular.z = self.yaw_pid.get_output(self.curr_pose[2], ref_yaw, 1 / self.freq)
                elif self.rot2goal_yaw:
                    rospy.loginfo_throttle(1, "Rotating to goal yaw")
                    ref_yaw = quat2yaw(self.goal_pose.pose.orientation)
                    ref_yaw = get_acute_angle(self.curr_pose[2], ref_yaw)
                    # rospy.loginfo(f"Rotating: {self.curr_pose[2]:.2f} -> {ref_yaw:.2f}")
                    if abs(ref_yaw - self.curr_pose[2]) < self.ang_tol:  # really reached the goal
                        msg = ReachGoal()
                        msg.result.data = True
                        msg.goal = self.goal_pose
                        self.reach_pub.publish(msg)
                        msg = Int8()
                        msg.data = NavStatus.ARRIVED
                        self.status_pub.publish(msg)
                        rospy.loginfo("Goal reached!")
                        self.rot2goal_yaw = False
                        self.enable_ctrl = False
                        continue
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0
                    cmd_vel.angular.z = self.yaw_pid.get_output(self.curr_pose[2], ref_yaw, 1 / self.freq)
                else:
                    rospy.loginfo_throttle(1, "Tracking path")
                    if self.method == "mpc":
                        cmd_vel = self.compute_cmd_vel_mpc()
                    elif self.method == "lqr":
                        cmd_vel = self.compute_cmd_vel_lqr()
            else:
                cmd_vel = Twist()
                status = Int8()
                status.data = NavStatus.IDLE
                self.status_pub.publish(status)
            # self.pub_cmd_vel.publish(cmd_vel)
            rate.sleep()


if __name__ == "__main__":
    try:
        path_tracker_node = NMPCNode(method="mpc")
        path_tracker_node.run()
    except rospy.ROSInterruptException:
        pass
