import rospy
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String


def is_goal_reached(goal_pose_stamped, robot_pose_stamped, margin=0.1):
    if robot_pose_stamped is None or goal_pose_stamped is None:
        return False
    if abs(goal_pose_stamped.pose.position.x - robot_pose_stamped.pose.position.x) < margin and abs(goal_pose_stamped.pose.position.y - robot_pose_stamped.pose.position.y) < margin:
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
            self.robot.set_state(self.robot.task1_to_task2_state, None)
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
            rospy.loginfo("Goal Reached")
            if self.curr_phase == 0:
                self.curr_phase = 1
                self.goal_pose = self.robot.get_goal_pose_from_config_map("/task1_complete_2")
                self.robot.pub_goal.publish(self.goal_pose)
                self.robot.goal_reached = False
            elif self.curr_phase == 1:
                self.curr_phase = 2
                self.goal_pose = self.robot.get_goal_pose_from_config_map("/task1_crossing_1")
                self.robot.pub_goal.publish(self.goal_pose)
                self.robot.goal_reached = False
            elif self.curr_phase == 2:
                # self.curr_phase = 3
                self.robot.pub_percep_cmd.publish("red")
                self.robot.percept_wait = "red"
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
        if is_goal_reached(self.goal_pose, self.robot.robot_pose):
            rospy.loginfo("Goal Reached")
            self.robot.set_state(self.robot.idle_state)
        pass
    

class Task2State(State):
    def __init__(self, robot):
        super().__init__(robot)
        self.process = None
        self.curr_phase = 0

    def init(self, arg):
        self.robot.pub_explore.publish(True)
        self.robot.pub_percep_cmd.publish("number")
        self.robot.percept_wait = "number"
        self.robot.number_pose = None
        self.curr_phase = 0


    def execute(self):
        if self.curr_phase == 0:
            if self.robot.number_pose is not None:
                # self.robot.pub_percep_cmd.publish("idle")
                # self.robot.percept_wait = ""
                if is_goal_reached(self.robot.number_pose, self.robot.robot_pose, 0.2):
                    rospy.loginfo("Goal Reached")
                    self.curr_phase = 1
                    self.robot.pub_percep_cmd.publish("idle")
                    self.robot.percept_wait = ""
                    self.robot.pub_goal.publish(self.robot.robot_pose)

                self.robot.pub_goal.publish(self.robot.number_pose)
                self.robot.number_pose = None
            
        elif self.curr_phase == 1:
            pass

    def terminate(self):
        self.robot.percept_wait = ""
        self.robot.pub_explore.publish(False)

class Task3Tracking(State):
    def init(self, goal_pose):
        self.robot.pub_goal.publish(goal_pose)

    def execute(self):
        pass

class IdleState(State):
    def init(self, args=None):
        self.robot.pub_goal.publish(self.robot.robot_pose)

    def execute(self):
        pass