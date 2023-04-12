import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from irobot_create_msgs.msg import HazardDetectionVector
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

    
from geometry_msgs.msg import Twist
from rclpy import qos
import random
import time

# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils

class Minesweeper(Node):

    def __init__(self):
        super().__init__('minesweeper')
        self.video_feed = self.create_subscription(
            Image,
            'camera/color/image_raw',
            self.video_callback,
            qos.qos_profile_sensor_data)
        
        self.bridge = CvBridge()
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID') # Specify video codec
        self.out = cv2.VideoWriter('output.avi', self.fourcc, 30.0, (640, 480)) # Create VideoWriter object

    def video_callback(self, msg):
        # Convert ROS2 Image message to cv2 image format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Write the frame to the video file
        self.out.write(cv_image)
        # Display the frame using cv2.imshow
        cv2.imshow('Video Stream', cv_image)
        cv2.waitKey(1)
#ros2 topic pub qbert/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
def main(args=None):
    rclpy.init(args=args)

    minesweeper = Minesweeper()
    
    rclpy.spin(minesweeper)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minesweeper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



