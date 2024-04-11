#!/usr/bin/env python3

import sys

import ipdb
import rospy
import tf
import tf2_geometry_msgs
import tf2_ros
import tf_conversions
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Odometry
from states import IdleState, Task1ToTask2, Task1Tracking, Task2Entry, Task2State, Task3Tracking
from std_msgs.msg import Bool, String

from final_pnc.msg import ReachGoal

sys.path.append(sys.path[0] + "/../../final_pnc/src/")
from utils import euclidian_dist_se2, ndarray2pose_se2, pose2ndarray_se2

init_flag = True
init_arg = None

# determine whether the robot have reached the goal
# determine whether the robot have reached the goal
turning_points = [(4.75, -6.5, 1), (4.75, 1.2, 1.5), (0.7, 2.7, 1.5), (11.5, 3.5, 1)]
vel_waypoints = [
    [0, 0, 2.2],
    [4.27, -6.48, 1.8],
    [5.59, -5.25, 2],
    [5.71, 0.4, 1.8],
    [0.34, 3.46, 3],
    [12.2, 3.43, 1.8],
    [12.2, 3.43, 1.8],
]  # x,y,vel


def euclidian_dist(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5


def near_turning_points(robot_pose_stamped, turning_points):
    if robot_pose_stamped is None or turning_points is None:
        return False
    for point in turning_points:
        if (
            abs(point[0] - robot_pose_stamped.pose.position.x) < point[2]
            and abs(point[1] - robot_pose_stamped.pose.position.y) < point[2]
        ):
            return True
    return False


class Robot:
    def __init__(self):
        self.robot_pose = None
        self.goal_reached = False

        # Initialization
        self.robot_frame = "base_link"
        self.map_frame = "map"
        self.world_frame = "world"
        self.robot_odom = PoseStamped()
        self.pose_map_robot = PoseStamped()

        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        # self.tf_listener = tf.TransformListener()
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.sub_goal_reached = rospy.Subscriber("/final_pnc/reach_goal", ReachGoal, self.goal_reached_callback)
        # self.pub_goal_name = rospy.Publisher("/rviz_panel/goal_name", String, queue_size=1)

        self.ref_vel_pub = rospy.Publisher("/final_pnc/set_ref_vel", Twist, queue_size=1)
        self.sub_robot_odom = rospy.Subscriber("/final_slam/odom", Odometry, self.robot_odom_callback)
        self.sub_goal_name = rospy.Subscriber("/rviz_panel/goal_name", String, self.goal_name_callback)
        self.sub_goal_pose = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_pose_callback)

        # Publisher and subscriber for perception
        self.percept_wait = ""
        self.pub_percep_cmd = rospy.Publisher("/percep/cmd", String, queue_size=1)
        self.sub_percept_red = rospy.Subscriber("/percep/red", String, self.percep_red_callback)
        self.sub_percpet_number_pose = rospy.Subscriber("/percep/pose", PoseStamped, self.percep_number_pose_callback)
        self.sub_percept_number = rospy.Subscriber("/percep/numbers", String, self.percep_number_callback)
        self.number_pose = None

        self.pub_explore = rospy.Publisher("/start_explore", Bool, queue_size=1)

        # State Machine
        self.idle_state = IdleState(self)
        self.task1_state = Task1Tracking(self)
        self.task1_to_task2_state = Task1ToTask2(self)
        self.task2_entry_state = Task2Entry(self)
        self.task2_state = Task2State(self)
        self.task3_state = Task3Tracking(self)
        self.current_state = self.idle_state

    def set_state(self, state, args=None):
        self.current_state.terminate()
        self.current_state = state
        state.init(args)

    def init_state(self, args=None):
        self.current_state.init(args)

    def execute_state(self):
        self.current_state.execute()

    def goal_name_callback(self, name):
        goal_name = name.data
        end = goal_name.find("_")
        self.goal_type = goal_name[1:end]
        # goal_box_id = int(goal_name[end+1])
        print("Goal Type: ", self.goal_type)

        P_world_goal = PoseStamped()
        if self.goal_type == "box":
            # TODO: Get box poses from gazebo
            pass
        else:
            # Get the Pose of the goal in world frame
            P_map_goal = self.get_goal_pose_from_config_map(goal_name)

        # Publish goal pose in map frame
        if self.goal_type == "assembly":
            self.set_state(self.task1_state, P_map_goal)
        elif self.goal_type == "vehicle":
            self.set_state(self.task3_state, P_map_goal)
        elif self.goal_type == "box":
            self.set_state(self.task1_to_task2_state, None)
        elif self.goal_type == "task1_complete":
            self.set_state(self.task1_to_task2_state, None)
        else:
            rospy.logwarn("Invalid goal type: %s", self.goal_type)

    def robot_odom_callback(self, odom: Odometry):
        self.world_frame = odom.header.frame_id
        self.robot_frame = odom.child_frame_id
        self.robot_odom.pose = odom.pose.pose
        self.robot_odom.header = odom.header

    def goal_pose_callback(self, goal_pose):
        self.pose_map_goal = goal_pose

    def percep_red_callback(self, data):
        if self.percept_wait != "red":
            return

        if data.data == "true":
            self.set_state(self.task2_entry_state, True)
            self.percept_wait = ""
        elif data.data == "false":
            self.set_state(self.task2_entry_state, False)
            self.percept_wait = ""

    def percep_number_pose_callback(self, pose):
        if self.percept_wait != "number":
            return
        # Get the number pose in map frame
        self.number_pose = pose
        rospy.loginfo_throttle(1, "Received number box pose!")
        # self.percept_wait = ""

    def percep_number_callback(self, data):
        if self.percept_wait != "number":
            return
        if data.data == "number found":
            self.pub_explore.publish(False)

    def get_goal_pose_from_config_map(self, name):
        P_world_goal = self.get_goal_pose_from_config(name)
        self.pose_world_goal = P_world_goal.pose

        # Get the Transform from world to map from the tf_listener
        try:
            transform_map_world = self.tf2_buffer.lookup_transform(self.map_frame, self.world_frame, rospy.Time(0))
            # self.tf_listener.waitForTransform(self.map_frame, self.world_frame, rospy.Time(0), rospy.Duration(1.0))
            # transform_map_world = self.tf_listener.lookupTransform(self.map_frame, self.world_frame, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rospy.logwarn("Failed to lookup transform: %s", str(ex))
            return

        # Transform the goal pose to map frame
        P_map_goal = PoseStamped()
        P_map_goal.header.stamp = rospy.Time.now()
        P_map_goal.header.frame_id = self.map_frame
        P_map_goal = tf2_geometry_msgs.do_transform_pose(P_world_goal, transform_map_world)
        # self.tf_listener.waitForTransform(self.map_frame, self.world_frame, rospy.Time(0), rospy.Duration(1.0))
        # P_map_goal = self.tf_listener.transformPose(self.map_frame, P_world_goal)

        # Transform the robot pose to map frame
        self.pose_map_robot = tf2_geometry_msgs.do_transform_pose(self.robot_odom, transform_map_world)
        # self.tf_listener.waitForTransform(self.map_frame, self.world_frame, rospy.Time(0), rospy.Duration(1.0))
        # self.pose_map_robot = self.tf_listener.transformPose(self.map_frame, self.robot_odom)
        return P_map_goal

    def get_goal_pose_from_config(self, name):
        """
        Get the Transform from goal to world from the file
        """
        print("/me5413_world" + name + "/x")
        x = rospy.get_param("/me5413_world" + name + "/x")
        y = rospy.get_param("/me5413_world" + name + "/y")
        yaw = rospy.get_param("/me5413_world" + name + "/yaw")
        self.world_frame = rospy.get_param("/me5413_world/frame_id")

        q = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw)

        P_world_goal = PoseStamped()
        P_world_goal.header.stamp = rospy.Time.now()
        P_world_goal.header.frame_id = self.world_frame
        P_world_goal.pose.position.x = x
        P_world_goal.pose.position.y = y
        P_world_goal.pose.orientation.x = q[0]
        P_world_goal.pose.orientation.y = q[1]
        P_world_goal.pose.orientation.z = q[2]
        P_world_goal.pose.orientation.w = q[3]

        return P_world_goal

    def goal_reached_callback(self, data):
        self.goal_reached = data.result.data


if __name__ == "__main__":
    global robot
    rospy.init_node("robot_state_machine", anonymous=True)
    # ipdb.set_trace()

    robot = Robot()
    vel_waypoint_idx = 0
    rate = rospy.Rate(20)  # 10hz
    while not rospy.is_shutdown():

        robot.execute_state()

        if robot.percept_wait == "red":
            robot.pub_percep_cmd.publish(robot.percept_wait)
        elif robot.percept_wait == "number":
            robot.pub_percep_cmd.publish(robot.percept_wait)
        else:
            robot.pub_percep_cmd.publish("idle")

        # publish the speed command
        ref_vel = Twist()
        if robot.current_state == robot.task2_state:
            ref_vel.linear.x = 1.5
        elif robot.current_state == robot.task3_state:
            ref_vel.linear.x = 2.2
        elif robot.current_state != robot.idle_state:
            vel = vel_waypoints[vel_waypoint_idx][2]
            if robot.robot_odom is not None:
                dist = euclidian_dist(pose2ndarray_se2(robot.robot_odom.pose), vel_waypoints[vel_waypoint_idx + 1])
                if dist < 0.5:
                    vel_waypoint_idx += 1
                    if vel_waypoint_idx >= len(vel_waypoints) - 1:
                        vel_waypoint_idx = -1
            rospy.loginfo_throttle(2, f"vel_waypoint_idx: {vel_waypoint_idx}, vel: {vel}")
            ref_vel.linear.x = vel
        else:
            ref_vel.linear.x = 2.0
        robot.ref_vel_pub.publish(ref_vel)

        rate.sleep()
