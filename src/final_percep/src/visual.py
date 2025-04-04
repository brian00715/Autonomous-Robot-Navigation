#!/usr/bin/env python3

# Create a node to subscribe to the topic from the camera
# Process the image with ocr and get the result bounding boxes
# Calculate the angle of the object with the parameters of the camera
# Publish the result to new topic

import time

import cv2
import easyocr
import ipdb
import numpy as np
import rospy
import tf
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

# import torch

# device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
# print(f"==>> device: {device}")


class Visual:
    def __init__(self, rate=10):
        rospy.loginfo(f"running rate: {rate}")
        self.noi = "1"  # number of interest
        self.detect_mode = "number"
        self.bridge = CvBridge()
        self.rate = rate

        self.img_curr = None
        self.img_curr_gray = None
        self.num_detect_result = [0] * 10  # 1 for detected, 0 for not detected
        self.camera_info = rospy.wait_for_message("/front/camera_info", CameraInfo)
        self.intrinsic = np.array(self.camera_info.K).reshape(3, 3)
        self.projection = np.array(self.camera_info.P).reshape(3, 4)
        self.distortion = np.array(self.camera_info.D)
        self.img_frame = self.camera_info.header.frame_id
        self.ocr_detector = easyocr.Reader(["en"], gpu=True)
        self.numberposelists = np.zeros((10, 2))
        self.curr_odom = None

        # pubs
        self.target_pose_pub = rospy.Publisher("/percep/pose", PoseStamped, queue_size=1)
        self.pubid = rospy.Publisher("/percep/numbers", String, queue_size=1)
        self.red_pub = rospy.Publisher("/percep/red", String, queue_size=1)
        # subs
        # self.mapsub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.tf_sub = tf.TransformListener()
        self.scansub = rospy.Subscriber("/front/scan", LaserScan, self.scan_callback)
        self.goalsub = rospy.Subscriber("/rviz_panel/goal_name", String, self.goal_callback)
        self.redcmdsub = rospy.Subscriber("/percep/cmd", String, self.detect_mode_callback)
        self.img_sub = rospy.Subscriber("/front/image_raw", Image, self.img_callback)
        self.odom_sub = rospy.Subscriber("/final_slam/odom", Odometry, self.odom_callback)
        rospy.loginfo("visual node initialized")

    def odom_callback(self, msg):
        self.curr_odom = msg

    def img_callback(self, msg: Image):
        self.img_curr = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def detect_mode_callback(self, msg):
        self.detect_mode = msg.data

    def map_callback(self, msg):
        data = list(msg.data)
        for y in range(msg.info.height):
            for x in range(msg.info.width):
                i = x + (msg.info.height - 1 - y) * msg.info.width
                if data[i] >= 75:
                    data[i] = 100
                elif (data[i] >= 0) and (data[i] < 50):  # free
                    data[i] = 0
                else:  # unknown
                    data[i] = -1
        self.map = np.array(data).reshape(msg.info.height, msg.info.width)

    def goal_callback(self, msg):
        if msg.data[:4] == "/box":
            self.noi = msg.data[-1]
            rospy.loginfo(f"number of interest: {self.noi}")

    def scan_callback(self, msg: LaserScan):
        self.scan_curr = msg.ranges
        self.scan_params_curr = [msg.angle_min, msg.angle_max, msg.angle_increment]

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.img_curr is None:
                continue
            rospy.loginfo_throttle(2, f"detect_mode: {self.detect_mode}")

            if self.detect_mode == "red":
                hsv_image = cv2.cvtColor(self.img_curr, cv2.COLOR_BGR2HSV)
                lower_red = np.array([0, 100, 100])
                upper_red = np.array([10, 255, 255])
                mask = cv2.inRange(hsv_image, lower_red, upper_red)
                if np.any(mask != 0):
                    self.red_pub.publish("true")
                else:
                    self.red_pub.publish("false")
            elif self.detect_mode == "number":
                if self.img_curr is None:
                    continue
                # img_gray = cv2.cvtColor(self.img_curr, cv2.COLOR_BGR2GRAY)
                # img_bin = cv2.threshold(img_gray, 20, 255, cv2.THRESH_BINARY)[1]
                # cv2.imshow("img_bin", img_bin)
                # cv2.waitKey(1)
                result = self.ocr_detector.readtext(self.img_curr, batch_size=2, allowlist="0123456789")
                img_show = self.img_curr.copy()
                for detection in result:
                    # detection[0]: the bounding box of the detected text
                    # detection[1]: the detected text
                    # detection[2]: the confidence of the detected text
                    if len(detection[1]) > 1:  # not a single digit
                        continue
                    diag_vec = np.array(detection[0][2]) - np.array(detection[0][0])
                    diag_len = np.linalg.norm(diag_vec)
                    if 1:
                        cv2.rectangle(
                            img_show,
                            (int(detection[0][0][0]), int(detection[0][0][1])),
                            (int(detection[0][2][0]), int(detection[0][2][1])),
                            (0, 255, 0),
                            2,
                        )
                        cv2.putText(
                            img_show,
                            detection[1] + f" {detection[2]:.2f}",
                            (int(detection[0][0][0]), int(detection[0][0][1])),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1,
                            (0, 255, 0),
                            2,
                            cv2.LINE_AA,
                        )
                        cv2.putText(
                            img_show,
                            f"diag: {diag_len:.2f}",
                            (int(detection[0][0][0]), int(detection[0][0][1] - 40)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            2,
                            cv2.LINE_AA,
                        )
                        cv2.imshow("img", img_show)
                        cv2.waitKey(1)

                    if detection[2] < 0.99:
                        continue
                    if (
                        diag_len < 60
                    ):  # prevent the case that  Recognizing 1 too early leads to incorrect distance estimation
                        continue
                    if detection[1] != self.noi:
                        continue
                    # Get the conter of the bounding box in [x, y] format
                    center = [(x + y) / 2 for x, y in zip(detection[0][0], detection[0][2])]
                    # Get the direction of the point in camera frame, [u, v, 1]
                    direction = np.array([[center[0]], [center[1]], [1]])

                    # Remap the direction to camera frame
                    direction = np.dot(np.linalg.inv(self.intrinsic), direction)

                    p_in_cam = PoseStamped()
                    p_in_cam.header.frame_id = self.img_frame
                    p_in_cam.pose.position.x = direction[0].item()
                    p_in_cam.pose.position.y = direction[1].item()
                    p_in_cam.pose.position.z = direction[2].item()
                    p_in_cam.pose.orientation.w = 1

                    # Transform the direction to other frame
                    self.tf_sub.waitForTransform("tim551", self.img_frame, rospy.Time.now(), rospy.Duration(1))
                    transformed = self.tf_sub.transformPose("tim551", p_in_cam)

                    # Calculate the yaw of the transformed result in current frame
                    yaw = np.arctan2(transformed.pose.position.y, transformed.pose.position.x)
                    # Calculate the nearest point in the scan
                    idx = round((yaw - self.scan_params_curr[0]) / self.scan_params_curr[2])
                    # Calculate the position of the nearest point in the scan
                    distance = self.scan_curr[idx] - 0.6  # minus certain distance
                    angle = self.scan_params_curr[0] + idx * self.scan_params_curr[2]
                    x = distance * np.cos(angle)
                    y = distance * np.sin(angle)
                    p_in_lidar = PoseStamped()
                    p_in_lidar.header.frame_id = "tim551"
                    p_in_lidar.header.stamp = rospy.Time.now()
                    p_in_lidar.pose.position.x = x
                    p_in_lidar.pose.position.y = y
                    p_in_lidar.pose.position.z = 0
                    p_in_lidar.pose.orientation.w = 1

                    self.tf_sub.waitForTransform("/map", "tim551", rospy.Time.now(), rospy.Duration(1))
                    p_in_map = self.tf_sub.transformPose("/map", p_in_lidar)
                    x = p_in_map.pose.position.x
                    y = p_in_map.pose.position.y
                    # Check if the point is valid
                    if x == np.inf or x == -np.inf or x == np.nan or y == np.inf or y == -np.inf or y == np.nan:
                        continue
                    if x > 17 or x < 7 or y > 2 or y < -7.2:
                        continue

                    # Save the position of the number
                    numberpose = np.array([x, y])
                    idx = int(detection[1])
                    self.numberposelists[idx] = (
                        numberpose
                        if self.numberposelists[idx, 0] == 0 and self.numberposelists[idx, 1] == 0
                        else self.numberposelists[idx] * 0.8 + numberpose * 0.2
                    )
                    self.num_detect_result[idx] = 1

                    # Pulish the direction and the id of the object
                    # self.pubpose.publish(transformed)
                    # self.pubid.publish(self.percepnums)
                    if self.num_detect_result[int(self.noi)] == 1:
                        self.pubid.publish(String(data=str("number found")))
                        goal_x = self.numberposelists[int(self.noi), 0]
                        goal_y = self.numberposelists[int(self.noi), 1]
                        goal_p = PoseStamped()
                        goal_p.header.frame_id = "map"
                        goal_p.header.stamp = rospy.Time.now()
                        goal_p.pose.position.x = goal_x
                        goal_p.pose.position.y = goal_y
                        goal_p.pose.position.z = 0
                        if self.curr_odom is not None:
                            goal_p.pose.orientation = self.curr_odom.pose.pose.orientation
                        else:
                            goal_p.pose.orientation.w = 1
                        self.target_pose_pub.publish(goal_p)

                        print(goal_x, goal_y)
                    # print(self.numberposelists)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("visual")
    v = Visual(rate=30)
    v.run()
