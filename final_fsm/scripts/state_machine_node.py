#!/usr/bin/env python3

import rospy
import roslaunch
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Bool
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
import threading, subprocess

init_flag = True
init_arg = None



class State:
    def __init__(self, robot):
        self.robot = robot
    def init(self):
        pass
    def execute(self):
        pass
    def terminate(self):
        pass

class Task1Tracking(State):
    def init(self, goal_pose):
        self.robot.pub_goal.publish(goal_pose)

    def execute(self):
        pass

class Task2State(State):
    def __init__(self, robot):
        super().__init__(robot)
        self.process = None

    def init(self, arg):
        pub_explore.publish(True)
        # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(uuid)
        # launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/luyi/catkin_ws/src/ME5413_Final_Project/final_fsm/explore.launch"])
        # launch.start()
        # self.process = launch

    def execute(self):
        pass
    def terminate(self):
        pub_explore.publish(False)

class Task3Tracking(State):
    def init(self, goal_pose):
        self.robot.pub_goal.publish(goal_pose)

    def execute(self):
        pass

class IdleState(State):
    def init(self):
        rospy.loginfo("机器人处于待机状态")

    def execute(self):
        pass

class Robot:
    def __init__(self):
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        self.sub_robot_odom = rospy.Subscriber("/gazebo/ground_truth/state", PoseStamped, self.robot_odom_callback)
        self.sub_goal_name = rospy.Subscriber("/rviz_panel/goal_name", String, self.goal_name_callback)
        self.sub_goal_pose = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_pose_callback)
        
        # Initialization
        self.robot_frame = "base_link"
        self.map_frame = "map"
        self.world_frame = "world"
        self.pose_world_robot = PoseStamped()
        self.pose_map_robot = PoseStamped()

        # State Machine
        self.idle_state = IdleState(self)
        self.task1_state = Task1Tracking(self)
        self.task2_state = Task2State(self)
        self.task3_state = Task3Tracking(self)
        self.current_state = self.idle_state


    def set_state(self, state, args=None):
        self.current_state.terminate()
        self.current_state = state
        global init_flag
        init_flag = False

        global init_arg
        init_arg = args
        # state.init(args)

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
            P_world_goal = self.get_goal_pose_from_config(goal_name)

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

        # Publish goal pose in map frame 
        if self.goal_type == "assembly":
            self.set_state(self.task1_state, P_map_goal)
        elif self.goal_type == "vehicle":
            self.set_state(self.task3_state, P_map_goal)
        elif self.goal_type == "box":
            self.set_state(self.task2_state)
        else:
            self.set_state(self.idle_state)
            rospy.logwarn("Invalid goal type: %s", self.goal_type)

    def robot_odom_callback(self, odom):
        self.world_frame = odom.header.frame_id
        self.robot_frame = odom.child_frame_id
        self.pose_world_robot = odom
    
    def goal_pose_callback(self, goal_pose):
        self.pose_map_goal = goal_pose

    def get_goal_pose_from_config(self, name):
        """ 
        Get the Transform from goal to world from the file
        """
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


def state_callback(data):
    global robot
    if data.data == "Complete":
        robot.set_state(robot.idle_state)


if __name__ == '__main__':
    global robot
    rospy.init_node('robot_state_machine', anonymous=True)
    robot = Robot()
    rospy.Subscriber("robot_state", String, state_callback)
    pub_explore = rospy.Publisher("/start_explore", Bool, queue_size=1)

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        if not init_flag:
            robot.init_state(init_arg)
            init_flag = True
            
        robot.execute_state()
        rate.sleep()