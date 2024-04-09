#!/usr/bin/env python3

import rospy
import roslaunch
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Twist
from std_msgs.msg import Bool
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
from states import IdleState, Task1Tracking, Task1ToTask2, Task2Entry, Task2State, Task3Tracking
from final_pnc.msg import ReachGoal

init_flag = True
init_arg = None

# determine whether the robot have reached the goal


def near_turning_points(robot_pose_stamped, turning_points, margin=0.1):
    if robot_pose_stamped is None or turning_points is None:
        return False
    for point in turning_points:
        if abs(point[0] - robot_pose_stamped.pose.position.x) < margin and abs(point[1] - robot_pose_stamped.pose.position.y) < margin:
            return True
    return False



class Robot:
    def __init__(self):
        self.robot_pose = None
        self.goal_reached = False
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.sub_goal_reached = rospy.Subscriber("/final_pnc/reach_goal", ReachGoal, self.goal_reached_callback)
        # self.pub_goal_name = rospy.Publisher("/rviz_panel/goal_name", String, queue_size=1)

        self.pub_vel = rospy.Publisher("/final_pnc/set_ref_vel", Twist, queue_size=1)
        self.sub_robot_odom = rospy.Subscriber("/gazebo/ground_truth/state", PoseStamped, self.robot_odom_callback)
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
        
        # Initialization
        self.robot_frame = "base_link"
        self.map_frame = "map"
        self.world_frame = "world"
        self.pose_world_robot = PoseStamped()
        self.pose_map_robot = PoseStamped()

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
        end = goal_name.find('_')
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
            self.set_state(self.task2_state)
        elif self.goal_type == "task1_complete":
            self.set_state(self.task1_to_task2_state, None)
            rospy.logwarn("Invalid goal type: %s", self.goal_type)
        else:
            rospy.logwarn("Invalid goal type: %s", self.goal_type)

    def robot_odom_callback(self, odom):
        self.world_frame = odom.header.frame_id
        self.robot_frame = odom.child_frame_id
        self.pose_world_robot = odom
    
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
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rospy.logwarn("Failed to lookup transform: %s", str(ex))
            return

        # Transform the goal pose to map frame
        P_map_goal = PoseStamped()
        P_map_goal = tf2_geometry_msgs.do_transform_pose(P_world_goal, transform_map_world)
        P_map_goal.header.stamp = rospy.Time.now()
        P_map_goal.header.frame_id = self.map_frame

        # Transform the robot pose to map frame
        self.pose_map_robot = tf2_geometry_msgs.do_transform_pose(self.pose_world_robot, transform_map_world)
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
        P_world_goal.pose.position.x = x
        P_world_goal.pose.position.y = y
        P_world_goal.pose.orientation.x = q[0]
        P_world_goal.pose.orientation.y = q[1]
        P_world_goal.pose.orientation.z = q[2]
        P_world_goal.pose.orientation.w = q[3]

        return P_world_goal

    def goal_reached_callback(self, data):
        self.goal_reached = data.result.data


if __name__ == '__main__':
    global robot
    rospy.init_node('robot_state_machine', anonymous=True)
    robot = Robot()
    # 订阅tf tree

    

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        try:
            transform = robot.tf2_buffer.lookup_transform(robot.map_frame, robot.robot_frame, rospy.Time(0))
            robot.robot_pose = tf2_geometry_msgs.do_transform_pose(robot.pose_world_robot, transform)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            pass
        
            
        robot.execute_state()

        if robot.percept_wait == "red":
            robot.pub_percep_cmd.publish(robot.percept_wait)
        elif robot.percept_wait == "number":
            robot.pub_percep_cmd.publish(robot.percept_wait)
        else:
            robot.pub_percep_cmd.publish("idle")

        # publish the speed command
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.5
        robot.pub_vel.publish(cmd_vel)

        rate.sleep()