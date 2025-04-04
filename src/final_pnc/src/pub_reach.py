#!/usr/bin/env python3

import rospy
from final_pnc.msg import ReachGoal

if __name__ == "__main__":
    rospy.init_node("pub_reach")
    pub = rospy.Publisher("/final_pnc/reach_goal", ReachGoal, queue_size=1)
    rospy.sleep(0.5)
    msg = ReachGoal()
    msg.result.data = True
    for i in range(3):
        pub.publish(msg)
