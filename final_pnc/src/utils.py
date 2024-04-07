"""
 # @ Author: Kuankuan Sima
 # @ Email: smkk00715@gmail.com
 # @ Create Time: 2024-04-06 18:42:59
 # @ Modified time: 2024-04-06 19:51:57
 # @ Description:
 """

import geometry_msgs.msg as geometry_msgs
import nav_msgs.msg as nav_msgs
import tf
import tf.transformations as tft
import numpy as np


def euclidian_dist_se2(p1: geometry_msgs.Pose, p2: geometry_msgs.Pose) -> float:
    return ((p1.position.x - p2.position.x) ** 2 + (p1.position.y - p2.position.y) ** 2) ** 0.5


def quat2yaw(q: geometry_msgs.Quaternion) -> float:
    return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def linear_interpolate_pose(pose1, pose2, t):
    """
    Linearly interpolates between two poses.

    Parameters:
    - pose1, pose2: geometry_msgs.msg.Pose, the start and end poses.
    - t: float, interpolation factor (0 <= t <= 1).

    Returns:
    - interpolated_pose: geometry_msgs.msg.Pose, the interpolated pose.
    """
    interpolated_pose = geometry_msgs.PoseStamped()
    interpolated_pose.pose.position.x = (1 - t) * pose1.pose.position.x + t * pose2.pose.position.x
    interpolated_pose.pose.position.y = (1 - t) * pose1.pose.position.y + t * pose2.pose.position.y
    interpolated_pose.pose.position.z = (1 - t) * pose1.pose.position.z + t * pose2.pose.position.z
    # Orientation is not interpolated in this example for simplicity.
    return interpolated_pose


def extract_interpolated_path(path: nav_msgs.Path, T, N, V):
    """
    Extracts and interpolates a new path based on time steps and reference speed from a nav_msgs.msg.Path.

    Parameters:
    - path: nav_msgs.msg.Path, the original path.
    - T: float, the forward recursive time step.
    - N: int, the number of points in the new path.
    - V: float, the reference speed.

    Returns:
    - new_path: nav_msgs.msg.Path, the new interpolated path.
    """
    if len(path.poses) < 1:
        return None
    new_path = nav_msgs.Path()
    new_path.header = path.header  # Copy the header from the original path
    total_distance = 0
    distances = []

    # Calculate distances between consecutive points and total distance.
    poses = path.poses  # List of PoseStamped
    for i in range(len(poses) - 1):
        dx = poses[i + 1].pose.position.x - poses[i].pose.position.x
        dy = poses[i + 1].pose.position.y - poses[i].pose.position.y
        dz = poses[i + 1].pose.position.z - poses[i].pose.position.z
        dist = np.sqrt(dx**2 + dy**2 + dz**2)
        distances.append(dist)
        total_distance += dist

    # Calculate step distance based on T, V, and total distance.
    step_distance = V * T
    target_distance = step_distance

    # Perform linear interpolation to generate new path.
    for _ in range(N):
        while distances and target_distance > distances[0]:
            target_distance -= distances.pop(0)
            poses.pop(0)

        if distances:
            t = target_distance / distances[0]
            new_pose = linear_interpolate_pose(poses[0], poses[1], t)
            new_path.poses.append(new_pose)
            target_distance += step_distance
        else:
            break

    # Ensure the new path has N points by copying the last point if necessary.
    if len(new_path.poses) > 0:
        while len(new_path.poses) < N:
            new_path.poses.append(new_path.poses[-1])  # Copy the last point if new path is shorter than expected.
    else:
        while len(new_path.poses) < N:
            new_path.poses.append(path.poses[0])
    return new_path


def ndarray2path(ar: np.ndarray) -> nav_msgs.Path:
    path = nav_msgs.Path()
    for i in range(ar.shape[0]):
        pose = geometry_msgs.PoseStamped()
        pose.pose.position.x = ar[i, 0]
        pose.pose.position.y = ar[i, 1]
        pose.pose.position.z = ar[i, 2]
        pose.pose.orientation.x = ar[i, 3]
        pose.pose.orientation.y = ar[i, 4]
        pose.pose.orientation.z = ar[i, 5]
        pose.pose.orientation.w = ar[i, 6]
        path.poses.append(pose)
    return path


def path2ndarray_se2(path: nav_msgs.Path) -> np.ndarray:
    """Path to ndarray with SE(2) representation."""
    ar = np.zeros((len(path.poses), 3))
    for i in range(len(path.poses)):
        ar[i, 0] = path.poses[i].pose.position.x
        ar[i, 1] = path.poses[i].pose.position.y
        euler = tft.euler_from_quaternion(
            [
                path.poses[i].pose.orientation.x,
                path.poses[i].pose.orientation.y,
                path.poses[i].pose.orientation.z,
                path.poses[i].pose.orientation.w,
            ]
        )
        ar[i, 2] = euler[2]
    return ar


def ndarray2pose_se2(ar: np.ndarray) -> geometry_msgs.Pose:
    pose = geometry_msgs.Pose()
    pose.position.x = ar[0]
    pose.position.y = ar[1]
    pose.position.z = 0
    q = tft.quaternion_from_euler(0, 0, ar[2])
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose


def pose2ndarray_se2(pose: geometry_msgs.Pose) -> np.ndarray:
    ar = np.zeros(3)
    ar[0] = pose.position.x
    ar[1] = pose.position.y
    euler = tft.euler_from_quaternion(
        [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
    )
    ar[2] = euler[2]
    return ar


if __name__ == "__main__":

    path = [
        [0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 1],
        [2, 0, 0, 0, 0, 0, 1],
        [3, 0, 0, 0, 0, 0, 1],
    ]
    path = ndarray2path(np.array(path))

    T = 0.5
    N = 10
    new_path = extract_interpolated_path(path, T, N, 0.5)
    for pose in new_path.poses:
        print(f"({pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.position.z})")
