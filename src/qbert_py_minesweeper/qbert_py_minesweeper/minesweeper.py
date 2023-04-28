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
        
        self.hazard_subscription = self.create_subscription(
            HazardDetectionVector,
            'qbert/hazard_detection',
            self.hazard_callback,
            qos.qos_profile_sensor_data)
        
        self.bridge = CvBridge()
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID') # Specify video codec
        self.out = cv2.VideoWriter('output.avi', self.fourcc, 30.0, (640, 480)) # Create VideoWriter object
        self.move = Twist()
        self.publisher = self.create_publisher(Twist, 'qbert/cmd_vel', 10)
        self.last_center = None
        self.boomed = 0

    def video_callback(self, msg):
        lower = (29, 86, 6)
        upper = (64, 255, 255)
        pts = deque()
        # Convert ROS2 Image message to cv2 image format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Write the frame to the video file
        self.out.write(cv_image)
        # Display the frame using cv2.imshow
        cv2.waitKey(1)
        
        frame = imutils.resize(cv_image, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cv2.imshow("Mask", mask)
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
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
            print(f"center:{center}")
        # update the points queue
        pts.appendleft(center)
            # loop over the set of tracked points
        for i in range(1, len(pts)):
            # if either of the tracked points are None, ignore
            # them
            if pts[i - 1] is None or pts[i] is None:
                continue
            # otherwise, compute the thickness of the line and
            # draw the connecting lines
            thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
            cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
        # show the frame to our screen
        cv2.imshow("Frame", frame)
        if center:
            self.last_center = center 
        self.tracking_callback()

    def tracking_callback(self):
        tracking_twist = Twist()
        if self.last_center:
            if self.last_center[0] < 225:
                tracking_twist.angular.z = .1
            elif self.last_center[0] > 375:
                tracking_twist.angular.z = -.1
            else:
                tracking_twist.linear.x = .075
            if self.last_center[1] >= 350:
                self.boomed += 1
                print("boomed")
                tracking_twist.angular.z = 0
                tracking_twist.linear.x = 0
        else:
            tracking_twist.linear.x = .075
        
        if self.boomed > 5:
            tracking_twist.angular.z = 0
            tracking_twist.linear.x = 0
            go_home()

        self.publisher.publish(tracking_twist)

    def hazard_callback(self, haz):
        pass
    
    def destroy_node(self):
        # Release the VideoWriter object
        self.out.release()
        # Close all windows
        cv2.destroyAllWindows()
        super().destroy_node()


class RedTapeDetector(Node):
    def __init__(self):
        super().__init__('red_tape_detector')
        self.subscription = self.create_subscription(
            Image,
            'camera/color/image_raw',
            self.video_callback,
            qos.qos_profile_sensor_data)
        self.subscription

    def image_callback(self, msg):
        bridge = CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Define the range of red color in BGR
        lower_red = (0, 0, 255)
        upper_red = (30, 30, 255)
        # Threshold the image to get only red pixels
        mask = cv2.inRange(bridge, lower_red, upper_red)
        # Find contours of red pixels
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # Check if red tape contours exist
        if len(contours) > 0:
            print("Red line found!")
        # Draw bounding boxes around red tape contours
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(bridge, (x, y), (x + w, y + h), (0, 255, 0), 2)
        # Display the image with bounding boxes
        cv2.imshow("Red tape detection", bridge)
        cv2.waitKey(1)

    # def go_home():
    #     move = Twist()
    #     # Find april tag
    #     # Move to april tag
    #     bridge = CvBridge()
    #     lower_red = (0, 0, 255)
    #     upper_red = (30, 30, 255)
    #         # Find red line
    #     mask = cv2.inRange(bridge, lower_red, upper_red)
    #     contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #     if len(contours) > 0:
    #         # Allign with red line
    #         x, y, w, h = cv2.boundingRect(contours[0])
    #         cv2.rectangle(bridge, (x, y), (x + w, y + h), (0, 255, 0), 2)
    #         # Move forward
        
    #     while 
                
    #         # Face April tag
    #     # Bump into april tag
    #     # Move back
    #     # Go home
    #     return

def main(args=None):
    rclpy.init(args=args)

    # minesweeper = Minesweeper()
    
    # rclpy.spin(minesweeper)
    # # Destroy the node explicitly
    # # (optional - otherwise it will be done automatically
    # # when the garbage collector destroys the node object)
    # minesweeper.destroy_node()

    tape = RedTapeDetector()
    rclpy.spin(tape)

    rclpy.shutdown()


if __name__ == '__main__':
    main()



