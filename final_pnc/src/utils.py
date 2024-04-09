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

class NavStatus(enumerate):
    """Navigation status enumeration."""
    IDLE = 0
    EXECUTING = 1
    FAILED = 2
    ARRIVED = 3

def euclidian_dist_se2(p1: geometry_msgs.Pose, p2: geometry_msgs.Pose) -> float:
    return ((p1.position.x - p2.position.x) ** 2 + (p1.position.y - p2.position.y) ** 2) ** 0.5


def quat2yaw(q: geometry_msgs.Quaternion) -> float:
    return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def lin_interp_pose(pose1, pose2, t):
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
    quat_interp = tft.quaternion_slerp(
        [pose1.pose.orientation.x, pose1.pose.orientation.y, pose1.pose.orientation.z, pose1.pose.orientation.w],
        [pose2.pose.orientation.x, pose2.pose.orientation.y, pose2.pose.orientation.z, pose2.pose.orientation.w],
        t,
    )
    interpolated_pose.pose.orientation.x = quat_interp[0]
    interpolated_pose.pose.orientation.y = quat_interp[1]
    interpolated_pose.pose.orientation.z = quat_interp[2]
    interpolated_pose.pose.orientation.w = quat_interp[3]
    return interpolated_pose


def path_lin_interpo_cut(path: nav_msgs.Path, T, N, V):
    """
    Extracts and interpolates a new path based on time steps and reference speed from a nav_msgs.msg.Path.

    Parameters:
    - path: nav_msgs.msg.Path, the original path.
    - T: float, the forward time step.
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
            new_pose = lin_interp_pose(poses[0], poses[1], t)
            new_path.poses.append(new_pose)
            target_distance += step_distance
        else:
            break

    # Ensure the new path has N points by copying the last point if necessary.
    if len(new_path.poses) > 0:
        while len(new_path.poses) < N:
            new_path.poses.append(new_path.poses[-1])  # Copy the last point if new path is shorter than expected.
    else:  # path is shorter than the forward time step
        while len(new_path.poses) < N:
            new_path.poses.append(path.poses[0])

    return new_path


def path_lin_interpo(original_path: nav_msgs.Path, step_dist: float) -> nav_msgs.Path:
    """
    Linearly interpolate between points in a path to ensure that the distance between
    consecutive points is approximately equal to step_dist.
    """
    # Create a new Path message to store the interpolated path
    interpolated_path = nav_msgs.Path()
    interpolated_path.header = original_path.header

    if not original_path.poses:
        return interpolated_path  # Return the empty path if the original is empty

    # Add the first pose of the original path to the interpolated path
    interpolated_path.poses.append(original_path.poses[0])

    for i in range(1, len(original_path.poses)):
        start_pose = original_path.poses[i - 1]
        end_pose = original_path.poses[i]

        # Calculate the distance between the current and the next pose
        distance = euclidian_dist_se2(start_pose.pose, end_pose.pose)

        # Calculate the number of steps needed between the current and next pose
        steps = int(np.ceil(distance / step_dist))

        for step in range(1, steps):
            # Calculate the interpolation factor
            t = step / steps

            # Interpolate positions
            interpolated_pose = geometry_msgs.PoseStamped()
            interpolated_pose.header = start_pose.header
            interpolated_pose.pose.position.x = (1 - t) * start_pose.pose.position.x + t * end_pose.pose.position.x
            interpolated_pose.pose.position.y = (1 - t) * start_pose.pose.position.y + t * end_pose.pose.position.y
            interpolated_pose.pose.position.z = (1 - t) * start_pose.pose.position.z + t * end_pose.pose.position.z

            # Optionally, interpolate orientations here as well

            # Add the interpolated pose to the path
            interpolated_path.poses.append(interpolated_pose)

        # Add the end pose of the current segment to the path
        interpolated_path.poses.append(end_pose)

    return interpolated_path


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


def ndarray2pose_stamp_se2(ar: np.ndarray) -> geometry_msgs.PoseStamped:
    pose = geometry_msgs.PoseStamped()
    pose.pose.position.x = ar[0]
    pose.pose.position.y = ar[1]
    pose.pose.position.z = 0
    q = tft.quaternion_from_euler(0, 0, ar[2])
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose


def get_acute_angle(ang1, ang2):
    """get acute angle w.r.t. ang1"""
    diff = ang2 - ang1
    while diff > np.pi:
        diff -= 2 * np.pi
    while diff < -np.pi:
        diff += 2 * np.pi
    return ang1 + diff


def find_nearest_point(path, curr_pose):
    if path is None or curr_pose is None:
        return None

    min_dist = float("inf")
    selected_idx = None
    robot_x, robot_y = curr_pose[0], curr_pose[1]
    robot_heading = curr_pose[2]

    for i, pose in enumerate(path.poses):
        point_x, point_y = pose.pose.position.x, pose.pose.position.y
        base2point_yaw = np.arctan2(point_y - robot_y, point_x - robot_x)
        dist = np.sqrt((point_x - robot_x) ** 2 + (point_y - robot_y) ** 2)

        # Calculate the absolute angle difference between robot heading and base2point_yaw
        angle_diff = abs(robot_heading - base2point_yaw)
        angle_diff = min(angle_diff, 2 * np.pi - angle_diff)  # Normalize angle to be within [0, π]

        # Threshold for angle difference can be adjusted. Here it's set to π/4 radians (45 degrees)
        if dist < min_dist:  # and angle_diff <= np.pi / 4:
            min_dist = dist
            selected_idx = i

    return selected_idx


def reorder_path_points(path):
    if len(path.poses) <= 1:
        return path

    # 获取路径中的第一个点作为参考点
    start_pose = path.poses[0]
    start_position = start_pose.pose.position

    # 计算每个点与参考点之间的距离
    distances = []
    for pose in path.poses[1:]:
        position = pose.pose.position
        dx = position.x - start_position.x
        dy = position.y - start_position.y
        distance = np.sqrt(dx**2 + dy**2)
        distances.append((pose, distance))

    # 根据距离对点进行排序
    sorted_poses = sorted(distances, key=lambda x: x[1])

    # 创建一个新的Path对象,并将排序后的点添加到其中
    reordered_path = nav_msgs.Path()
    reordered_path.header = path.header
    reordered_path.poses = [start_pose] + [pose for pose, _ in sorted_poses]

    return reordered_path


def find_point_from_idx_dist(path: nav_msgs.Path, idx, dist) -> geometry_msgs.PoseStamped:
    """find the point that is at specified distance from the origin in the path

    idx: int, the index of the point in the path
    dist: float, the distance from the idx-th point
    """
    if len(path.poses) == 0:
        return None
    if dist <= 0:
        return path.poses[idx]
    for i in range(idx, len(path.poses)):
        curr_point_dist = euclidian_dist_se2(path.poses[idx].pose, path.poses[i].pose)
        if curr_point_dist >= dist:
            return path.poses[i]

    return path.poses[-1]


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
    new_path = path_lin_interpo_cut(path, T, N, 0.5)
    for pose in new_path.poses:
        print(f"({pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.position.z})")
