from dynamic_reconfigure.client import Client as DynamicReconfigureClient
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String


def is_goal_reached(goal_pose_stamped, robot_pose_stamped, margin=0.1):
    if robot_pose_stamped is None or goal_pose_stamped is None:
        return False
    if (
        abs(goal_pose_stamped.pose.position.x - robot_pose_stamped.pose.position.x) < margin
        and abs(goal_pose_stamped.pose.position.y - robot_pose_stamped.pose.position.y) < margin
    ):
        return True
    return False


class State:
    def __init__(self, robot):
        self.robot = robot

    def init(self, args=None):
        pass

    def execute(self):
        pass

    def terminate(self):
        pass

    def _get_goal_reached(self):
        return self.robot.goal_reached


class Task1Tracking(State):
    def __init__(self, robot):
        super().__init__(robot)
        self.goal_pose = None

    def init(self, goal_pose):
        if goal_pose is None:
            # self.robot.set_state(self.robot.idle_state)
            return
        self.robot.pub_goal.publish(goal_pose)
        self.goal_pose = goal_pose
        self.robot.goal_reached = False

    def execute(self):
        # if is_goal_reached(self.goal_pose, self.robot.robot_pose):
        if self._get_goal_reached():
            rospy.loginfo("Goal Reached")
            self.robot.set_state(self.robot.idle_state, None)
        pass


class Task1ToTask2(State):
    def __init__(self, robot):
        super().__init__(robot)
        self.curr_phase = 0

    def init(self, args):
        # Start form phase 2
        self.curr_phase = 2
        self.goal_pose = self.robot.get_goal_pose_from_config_map("/task1_crossing_1")
        self.robot.pub_goal.publish(self.goal_pose)
        self.robot.goal_reached = False
        # self.robot.pub_goal_name.publish(String(data="/task1_complete_1"))

    def execute(self):
        # if is_goal_reached(self.goal_pose, self.robot.robot_pose):
        if self._get_goal_reached():
            rospy.loginfo("Task1ToTask2 Goal Reached")
            # if self.curr_phase == 0:
            #     self.curr_phase = 1
            #     self.goal_pose = self.robot.get_goal_pose_from_config_map("/task1_complete_2")
            #     self.robot.pub_goal.publish(self.goal_pose)
            #     self.robot.goal_reached = False
            # elif self.curr_phase == 1:
            #     self.curr_phase = 2
            #     self.goal_pose = self.robot.get_goal_pose_from_config_map("/task1_crossing_1")
            #     self.robot.pub_goal.publish(self.goal_pose)
            #     self.robot.goal_reached = False
            if self.curr_phase == 2:
                self.curr_phase = 3
                self.robot.pub_percep_cmd.publish("red")
                self.robot.percept_wait = "red"
        # else:
        #     if self.curr_phase == 2:
        #         self.goal_pose = self.robot.get_goal_pose_from_config_map("/task1_crossing_1")
        #         self.robot.pub_goal.publish(self.goal_pose)
        pass


class Task2Entry(State):
    def init(self, args):
        if args:
            self.goal_pose = self.robot.get_goal_pose_from_config_map("/task2_entry_1")
            self.robot.pub_goal.publish(self.goal_pose)
            self.robot.goal_reached = False
        else:
            self.goal_pose = self.robot.get_goal_pose_from_config_map("/task2_entry_2")
            self.robot.pub_goal.publish(self.goal_pose)
            self.robot.goal_reached = False

    def execute(self):
        if self._get_goal_reached():
            rospy.loginfo("Task2 Entry Goal Reached")
            # self.robot.set_state(self.robot.task2_state, None)
            self.robot.set_state(self.robot.task2_state, None)
        pass


class Task2State(State):
    def __init__(self, robot):
        super().__init__(robot)
        self.process = None
        self.curr_phase = 0
        self.sent_num_box_pose = False
        self.gcostmap_client = DynamicReconfigureClient("/move_base/global_costmap/", timeout=60)

    def init(self, arg):
        self.robot.pub_explore.publish(True)
        self.robot.pub_percep_cmd.publish("number")
        self.robot.percept_wait = "number"
        self.robot.number_pose = None
        self.curr_phase = 0
        self.robot.goal_reached = False
        self.gcostmap_client.update_configuration({"footprint": [[-0.05, -0.05], [-0.05, 0.05], [0.05, 0.05], [0.05, -0.05]]})

    def execute(self):
        if self.curr_phase == 0:
            if self.robot.number_pose is not None:  # detected the number
                # self.robot.pub_percep_cmd.publish("idle") # stop the perception
                # self.robot.percept_wait = ""

                # BUG: the task will be recognized as completed if the robot reaches the goal of expolre, not the real goal
                if is_goal_reached(self.robot.number_pose, self.robot.robot_pose, 0.3) or self._get_goal_reached():
                    rospy.loginfo("Task2State Goal Reached")
                    self.curr_phase = 1
                    self.robot.pub_percep_cmd.publish("idle")
                    self.robot.percept_wait = ""
                self.robot.pub_goal.publish(self.robot.number_pose)
                # if not self.sent_num_box_pose:
                #     self.robot.pub_goal.publish(self.robot.number_pose)
                #     self.sent_num_box_pose = True
                self.robot.number_pose = None
            # else:
            #     self.robot.pub_explore.publish(True)

        elif self.curr_phase == 1:
            pass

    def terminate(self):
        rospy.loginfo("Task2State Terminated")
        self.robot.percept_wait = ""
        self.robot.pub_explore.publish(False)
   

class Task3Tracking(State):
    def __init__(self, robot):
        super().__init__(robot)
        self.gcostmap_client = DynamicReconfigureClient("/move_base/global_costmap/", timeout=60)
    def init(self, goal_pose):
        self.robot.pub_goal.publish(goal_pose)
        self.gcostmap_client.update_configuration(
            {"footprint": [[-0.21, -0.165], [-0.21, 0.165], [0.21, 0.165], [0.21, -0.165]]}
        )
        self.robot.goal_reached = False

    def execute(self):
        if self._get_goal_reached():
            rospy.loginfo("Task3 Goal Reached")
            self.robot.set_state(self.robot.idle_state, None)
        pass


class IdleState(State):
    def init(self, args=None):
        rospy.loginfo("Enter Idle State")
        # self.robot.pub_goal.publish(self.robot.robot_pose)

    def execute(self):
        pass
