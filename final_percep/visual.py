#!/usr/bin/env python3

# Create a node to subscribe to the topic from the camera
# Process the image with ocr and get the result bounding boxes
# Calculate the angle of the object with the parameters of the camera
# Publish the result to new topic

import rospy
from sensor_msgs.msg import Image, CameraInfo, LaserScan
import easyocr
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf
import tf2_ros
from tf.transformations import euler_from_quaternion


class Visual:
    def __init__(self):
        rospy.init_node('visual')
        self.is_processing = False
        self.redcheck = False
        self.bridge = CvBridge()
        self.pubpose = rospy.Publisher('/percep/pose', PoseStamped, queue_size=10)
        self.pubid = rospy.Publisher('/percep/id', String, queue_size=10)
        self.camera_info = rospy.wait_for_message('/front/camera_info', CameraInfo)
        self.intrinsic = np.array(self.camera_info.K).reshape(3, 3)
        self.projection = np.array(self.camera_info.P).reshape(3, 4)
        self.distortion = np.array(self.camera_info.D)
        self.frame = self.camera_info.header.frame_id
        self.reader = easyocr.Reader(['en'])
        self.listener = tf.TransformListener()
        self.scansub = rospy.Subscriber('/front/scan', LaserScan, self.scan_callback)
        self.poselists = np.zeros((9, 2))
        # self.mapsub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.goalsub = rospy.Subscriber('/goal', String, self.goal_callback)
        self.sub = rospy.Subscriber('/front/image_raw', Image, self.callback)
        self.redcmdsub = rospy.Subscriber('/redcmd', String, self.redcmd_callback)
        self.redpub = rospy.Publisher('/percep/red', String, queue_size=10)
        rospy.spin()

    def callback(self, msg):
        if self.is_processing:
            return
        self.is_processing = True
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if self.redcheck:
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])
            mask = cv2.inRange(hsv_image, lower_red, upper_red)
            if np.any(mask!=0):
                self.redpub.publish('true')
            else:
                self.redpub.publish('false')
        else :
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            result = self.reader.readtext(gray)
            for detection in result:
                if len(detection[1]) > 1:
                    continue
                if detection[2] < 0.95:
                    continue
                if detection[1] < '1' or detection[1] > '9':
                    continue
                # Get the conter of the bounding box in [x, y] format
                center = [(x + y)/2 for x, y in zip(detection[0][0], detection[0][2])]
                # Get the direction of the point in camera frame, [u, v, 1]
                direction = np.array([[center[0]], [center[1]], [1]])

                # Remap the direction to camera frame
                direction = np.dot(np.linalg.inv(self.intrinsic), direction)


                # Create a PoseStamped message
                cur_p = PoseStamped()
                cur_p.header.frame_id = self.frame
                cur_p.pose.position.x = direction[0].item()
                cur_p.pose.position.y = direction[1].item()
                cur_p.pose.position.z = direction[2].item()
                cur_p.pose.orientation.w = 1

                # Transform the direction to other frame
                transformed = self.listener.transformPose('tim551', cur_p)

                

                # Calculate the yaw of the transformed result in current frame
                yaw = np.arctan2(transformed.pose.position.y, transformed.pose.position.x)
                # Calculate the nearest point in the scan
                idx = round((yaw - self.scanparams[0]) / self.scanparams[2])
                # Calculate the position of the nearest point in the scan
                distance = self.scan[idx]
                angle = self.scanparams[0] + idx * self.scanparams[2]
                x = distance * np.cos(angle)
                y = distance * np.sin(angle)
                point_p = PoseStamped()
                point_p.header.frame_id = 'tim551'
                point_p.pose.position.x = x
                point_p.pose.position.y = y
                point_p.pose.position.z = 0
                point_p.pose.orientation.w = 1

                # Pulish the direction and the id of the object
                #self.pubpose.publish(transformed)
                self.pubid.publish(detection[1])
                self.pubpose.publish(point_p)
            

            
        self.is_processing = False

    def redcmd_callback(self, msg):
        self.redcheck = True if msg.data == 'true' else False

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
        self.goal = msg.data

    def scan_callback(self, msg):
        self.scan = msg.ranges
        self.scanparams = [msg.angle_min, msg.angle_max, msg.angle_increment]



    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    v=Visual()
    v.run