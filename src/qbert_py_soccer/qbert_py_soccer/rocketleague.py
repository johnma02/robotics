import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
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

class RocketLeague(Node):
    def __init__(self):
        super().__init__('rocketleague')
        self.video_feed = self.create_subscription(
            Image,
            'camera/color/image_raw',
            self.video_callback,
            qos.qos_profile_sensor_data)

        self.subscription_ = self.create_subscription(
            Odometry, 'qbert/odom', self.receive_odometry,
            qos.qos_profile_sensor_data)

        self.april_subscription = self.create_subscription(
            AprilTagDetectionArray, 
            '/detections',
            self.april_callback,
            qos.qos_profile_sensor_data)
        
        self.bridge = CvBridge()
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID') # Specify video codec
        self.out = cv2.VideoWriter('output.avi', self.fourcc, 30.0, (640, 480)) # Create VideoWriter object

        self.scored = 0

        self.timer_stopper = None
        self.forward_timer = None

        self.setting_up = True

        self.last_center = None
        self.publisher = self.create_publisher(Twist, "qbert/cmd_vel",10)


    def forward_callback(self):
        forward = Twist()
        forward.linear.x = .1
        self.publisher.publish(forward)

    def turn_callback(self):
        tt = Twist() # tracking twist

    def tracking_callback(self):
        if(self.setting_up):
            return
        tt = Twist() # tracking twist

    def receive_odometry(self, odom):
        print(odom[orientation])

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


    def destroy_timers(self):
        if(self.forward_timer):
            self.destroy_timer(self.forward_timer)
        self.publisher.publish(Twist())
            
    def video_callback(self, msg):
        ap = argparse.ArgumentParser()
        ap.add_argument("-v", "--video",
        help="path to the (optional) video file")
        ap.add_argument("-b", "--buffer", type=int, default=16,
        help="max buffer size")
        args = vars(ap.parse_args())
        lower = (29, 100, 6)
        upper = (64, 255, 255)

        robot_lower = (0, 0, 0)
        robot_upper = (65, 65, 65)

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

        if center:
            self.last_center = center 
        self.tracking_callback()

    def destroy_node(self):
        # Release the VideoWriter object
        self.out.release()
        # Close all windows
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    rocketleague = RocketLeague()
    
    rclpy.spin(rocketleague)
    rocketleague.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()



