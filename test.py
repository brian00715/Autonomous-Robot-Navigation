import rospy
from dynamic_reconfigure.client import Client as DynamicReconfigureClient

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np

# fp1 = [[-0.21, -0.165], [-0.21, 0.165], [0.21, 0.165], [0.21, -0.165]]
# fp2 = [[-0.1, -0.1], [-0.1, 0.1], [0.1, 0.1], [0.1, -0.1]]
# if __name__ == "__main__":
#     rospy.init_node("test")
#     client = DynamicReconfigureClient("/move_base/global_costmap/", timeout=30)
#     rst = client.update_configuration({"footprint": fp1})
#     print(rst)
a= [1,2]
b =[2,3]
rest = np.linalg.norm(a)
print(f"==>> rest: {rest}")