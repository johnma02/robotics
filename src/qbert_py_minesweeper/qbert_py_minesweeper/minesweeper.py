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

    lower = (29, 86, 6)
    upper = (149, 234, 244)
    

    def __init__(self):
        super().__init__('minesweeper')
        self.video_feed = self.create_subscription(
            Image,
            'camera/color/image_raw',
            self.video_callback,
            qos.qos_profile_sensor_data)
        
        self.hazard_subscription = self.create_subscription(
            HazardDetectionVector,
            'qbert/hazard_detection',
            self.hazard_callback,
            qos.qos_profile_sensor_data)
        
        self.bridge = CvBridge()
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID') # Specify video codec
        self.out = cv2.VideoWriter('output.avi', self.fourcc, 30.0, (640, 480)) # Create VideoWriter object
        self.move = Twist()
        self.publisher = self.createPub(Twist, 'qbert/cmd_vel', 10)

    def video_callback(self, msg):
        # Convert ROS2 Image message to cv2 image format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Write the frame to the video file
        self.out.write(cv_image)
        # Display the frame using cv2.imshow
        cv2.imshow('Video Stream', cv_image)
        cv2.waitKey(1)

    def destroy_node(self):
        # Release the VideoWriter object
        self.out.release()
        # Close all windows
        cv2.destroyAllWindows()
        super().destroy_node()

    def getBallPosition(self):

        if not cv2:
            vs = VideoStream(src=0).start()
        # otherwise, grab a reference to the video file
        else:
            vs = cv2.VideoCapture(self.video_feed)

        # grab the current frame
        frame = vs.read()
        # handle the frame from VideoCapture or VideoStream
        frame = frame[1] if cv2 else frame
        # if we are viewing a video and we did not grab a frame,
        # then we have reached the end of the video
        if frame is None:
            return
        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, Minesweeper.lower, Minesweeper.upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
            # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            print((x,y))
            return (x,y)
        else:
            return None
    
    def getRot(self):
        try:
            x, y = Minesweeper.getBallPosition()
            diff = x-300
            if diff > 0:
                return 0.1
            else:
                return -0.1
        except:
            return 0
        
    def backUp(self):
        self.move.linear.x = -0.1
        self.move.angular.z = (random.random() - 0.5) * 2
        self.publisher.publish(self.move)
        
    def hazard_callback(self, haz):
        if len(haz.detections) > 0:
            types = [val.type for val in haz.detections] #list comprehension extracts types from list of objects
            if (1 in types) and not self.executing: #type 1 is collision
                self.backTimer = self.create_timer(0.5, self.backUp)
                self.executing = True
                


    def createPub(self, msg_type, topic, qos_profile):
        return self.create_publisher(msg_type, topic, qos_profile)
    
    def movement(self):
        rot = Minesweeper.getRot()
        self.move.linear.x = 0.3
        self.move.angular.z = rot
        self.publisher.publish(self.move)
    
    def run(self):
        while 1:
            Minesweeper.movement()

    
        


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



