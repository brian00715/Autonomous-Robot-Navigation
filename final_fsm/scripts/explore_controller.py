#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool

pause = True

def map_callback(msg):
    if pause:
        return
    # 该回调函数将接收到的map消息重新发布到map_explore话题
    pub_map.publish(msg)

def pause_callback(msg):
    global pause
    pause = not msg.data

if __name__ == '__main__':
    try:
        rospy.init_node('map_subscriber', anonymous=True)
        rospy.loginfo("Map subscriber node started")

        # 订阅map话题
        rospy.Subscriber('/map', OccupancyGrid, map_callback)

        rospy.Subscriber('/start_explore', Bool, pause_callback)

        # 创建一个发布者，用于发布重新封装的map消息到map_explore话题
        pub_map = rospy.Publisher('/map_explore', OccupancyGrid, queue_size=10)


        rospy.spin()
    except rospy.ROSInterruptException:
        pass