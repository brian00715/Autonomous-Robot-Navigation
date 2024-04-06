#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool

pause = True

# ***************
#   OBSTACLE
M = 70
#   unknown
N = 50
#   free
# ----0-----
#   unknown
# ***************


def map_callback(cmap: OccupancyGrid):
    if pause:
        return
    # 该回调函数将接收到的map消息重新发布到map_explore话题
    data = list(cmap.data)
    for y in range(cmap.info.height):
        for x in range(cmap.info.width):
            i = x + (cmap.info.height - 1 - y) * cmap.info.width
            if data[i] >= M:  
                data[i] = 100
            elif (data[i] >= 0) and (data[i] < N):  # free
                data[i] = 0
            else:  # unknown
                data[i] = -1
    cmap.data = tuple(data)
    pub_map.publish(cmap)

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