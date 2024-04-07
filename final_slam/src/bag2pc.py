import rosbag
import numpy as np
import struct

from sensor_msgs.msg import PointCloud2


def read_rosbag(bag_file, topic):
    bag = rosbag.Bag(bag_file, "r")
    for topic, msg, t in bag.read_messages(topics=[topic]):
        yield msg
    bag.close()


def convert_to_kitti_bin(msg, output_file):
    # assert isinstance(msg, PointCloud2)
    num_points = msg.width * msg.height
    dtype_list = [("x", np.float32), ("y", np.float32), ("z", np.float32), ("intensity", np.float32)]
    point_cloud = np.frombuffer(msg.data, dtype=dtype_list)

    # Ensure we're only using x, y, z, and reflectance
    kitti_points = np.zeros((num_points, 4), dtype=np.float32)
    kitti_points[:, 0] = point_cloud["x"]
    kitti_points[:, 1] = point_cloud["y"]
    kitti_points[:, 2] = point_cloud["z"]
    kitti_points[:, 3] = point_cloud["intensity"]

    # Save to .bin file
    kitti_points.tofile(output_file)


if __name__ == "__main__":
    bag_file = "/home/simon/LocalDiskExt/Datasets/HW2_SLAM/Task2/have_fun.bag"
    topic = "/kitti/velo/pointcloud"
    output_file = "/home/simon/LocalDiskExt/Datasets/HW2_SLAM/Task2/KITTI"

    idx = 0
    for msg in read_rosbag(bag_file, topic):
        filename = "%06d.bin" % idx
        path = output_file + "/" + filename
        convert_to_kitti_bin(msg, path)
        print(f"Saved LiDAR data to {path}")
        idx += 1
