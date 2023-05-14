import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from irobot_create_msgs.msg import HazardDetectionVector
from apriltag_msgs.msg import AprilTagDetectionArray 
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

# detector

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

        self.april_subscription = self.create_subscription(
            AprilTagDetectionArray, 
            '/detections',
            self.april_callback,
            qos.qos_profile_sensor_data)
        
        self.bridge = CvBridge()
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID') # Specify video codec
        self.out = cv2.VideoWriter('output.avi', self.fourcc, 30.0, (640, 480)) # Create VideoWriter object

        self.boomed = 0
        self.booming = False

        self.timer_stopper = None
        self.forward_timer = None

        self.heading_home = False
        self.last_center = None

    def forward_callback(self):
        forward = Twist()
        forward.linear.x = .1
        self.publisher.publish(forward)

    def destroy_forward(self):
        self.destroy_timer(self.forward_timer)
        self.destroy_timer(self.timer_stopper)
        self.booming = False
        self.boomed += 1
        print("boomed")
        self.last_center = None
        self.publisher.publish(Twist())

    def video_callback(self, msg):
        lower = (29, 100, 6)
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
            if radius > 12:
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
        if not self.heading_home:
            if center:
                self.last_center = center 
            self.tracking_callback()

    def tracking_callback(self):
        tracking_twist = Twist()
        if not self.heading_home:
            if self.last_center:
                if self.last_center[0] < 225:
                    tracking_twist.angular.z = .1
                elif self.last_center[0] > 375:
                    tracking_twist.angular.z = -.1
                else:
                    tracking_twist.linear.x = .075
                if self.last_center[1] >= 350:
                    if not self.booming:
                        self.forward_timer = self.create_timer(.1, self.forward_callback)
                        self.timer_stopper = self.create_timer(3, self.destroy_forward)
                        self.booming = True
                        print("booming")
            else:
                tracking_twist.angular.z = 0.1
            if self.boomed >= 4:
                self.heading_home = True
                self.last_center = None
        else:
            if self.last_center:
                if self.last_center[0] < 175:
                    tracking_twist.angular.z = .1
                elif self.last_center[0] > 250:
                    tracking_twist.angular.z = -.1
                else:
                    #DRIBBLE HERE

                    # if ball is in front of robot
                    if self.last_center[0] > 275 and self.last_center[0] < 325:
                        tracking_twist.linear.x = .075
                    # if ball is left of robot
                    if self.last_center[0] < 275:
                        # twist -0.1
                        # move forward
                        tracking_twist.angular.z = -.1
                        tracking_twist.linear.x = .075
                    # if ball is right of robot
                    if self.last_center[0] > 325:
                        # twist 0.1
                        # move forward
                        tracking_twist.angular.z = .1
                        

                if self.last_center[1] >= 425:
                    if not self.booming:
                        self.forward_timer = self.create_timer(.1, self.forward_callback)
                        self.timer_stopper = self.create_timer(3, self.destroy_forward)
                        self.booming = True
                        print("booming")
        if not self.last_center:
            tracking_twist.angular.z = 0.005

        self.publisher.publish(tracking_twist)

    def april_callback(self, detections):
        if self.heading_home:
            if(detections.detections):
                if(detections.detections[0].id == 8):
                    x = detections.detections[0].centre.x
                    y = detections.detections[0].centre.y
                    self.last_center = [x, y]
                    print(f"here: {self.last_center}")
                elif self.heading_home:
                    self.last_center = None
            self.tracking_callback()
            
    def hazard_callback(self, haz):
        pass

    def destroy_node(self):
        # Release the VideoWriter object
        self.out.release()
        # Close all windows
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    minesweeper = Minesweeper()
    
    rclpy.spin(minesweeper)
    minesweeper.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()



