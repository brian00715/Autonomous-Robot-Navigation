# Create a node to subscribe to the topic from the camera
# Process the image with ocr and get the result bounding boxes
# Calculate the angle of the object with the parameters of the camera
# Publish the result to new topic

import rospy
from sensor_msgs.msg import Image, CameraInfo
import easyocr
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf
from tf.transformations import euler_from_quaternion


class Visual:
    def __init__(self):
        rospy.init_node('visual')
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
        self.sub = rospy.Subscriber('/front/image_raw', Image, self.callback)
        self.is_processing = False
        rospy.spin()

    def callback(self, msg):
        if self.is_processing:
            return
        self.is_processing = True
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        result = self.reader.readtext(gray)
        for detection in result:
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
            transformed = self.listener.transformPose('map', cur_p)

            # Pulish the direction and the id of the object
            self.pubpose.publish(transformed)
            self.pubid.publish(detection[1])
        self.is_processing = False



    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    v=Visual()
    v.run