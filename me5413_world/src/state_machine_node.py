#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
from std_msgs.msg import Float32

class State:
    def __init__(self, robot):
        self.robot = robot

    def execute(self):
        pass

class IdleState(State):
    def execute(self):
        rospy.loginfo("机器人处于待机状态")

class MoveState(State):
    def execute(self):
        rospy.loginfo("机器人正在移动")

class StopState(State):
    def execute(self):
        rospy.loginfo("机器人停止移动")

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
        
        self.idle_state = IdleState(self)
        self.move_state = MoveState(self)
        self.stop_state = StopState(self)
        self.current_state = self.idle_state


    def set_state(self, state):
        self.current_state = state

    def execute_state(self):
        self.current_state.execute()

    def goal_name_callback(self, name):
        goal_name = name.data
        end = goal_name.find('_')
        self.goal_type = goal_name[1:end]
        # goal_box_id = int(goal_name[end+1])

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
        if self.goal_type != "box":
            self.pub_goal.publish(P_map_goal)

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
    if data.data == "MOVE":
        robot.set_state(robot.move_state)
    elif data.data == "STOP":
        robot.set_state(robot.stop_state)
    elif data.data == "IDLE":
        robot.set_state(robot.idle_state)



if __name__ == '__main__':
    global robot
    rospy.init_node('robot_state_machine', anonymous=True)
    robot = Robot()
    rospy.Subscriber("robot_state", String, state_callback)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        # robot.execute_state()
        rate.sleep()