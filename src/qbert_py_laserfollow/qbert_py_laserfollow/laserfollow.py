import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from irobot_create_msgs.msg import HazardDetectionVector
from sensor_msgs.msg import LaserScan
    
from geometry_msgs.msg import Twist
from rclpy import qos
import random
import time
import math

class Laserfollow(Node):

    def __init__(self):
        super().__init__("laserfollow")


        self.hazard_subscription = self.create_subscription(
                HazardDetectionVector,
                'qbert/hazard_detection',
                self.hazard_callback,
                qos.qos_profile_sensor_data)

        self.hazard_subscription

        self.laser_subscription = self.create_subscription(LaserScan,
                '/qbert/scan', 
                self.laser_callback, 
                qos.qos_profile_sensor_data)

        self.laser_subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(Twist, "qbert/cmd_vel",10)
        #in wall-following mode
        self.following = False
        
    def hazard_callback(self, haz):
        pass
    
    """
    if we aren't following a wall, we go forward until we detect enough evidence for a wall:
        0.4 meters
        -> determine if the robot is in a corner
        -> veer away from the wall but not too far
            -> publish twists with angular z's
        -> if we mess up, we stop and turn away from the wall

        For example: assuming an object is on the left, we correct based on whether or not lidar sensors readings are
        increasing in intensity or decreasing


        (1) Start in free space, go straight until a blocking wall is encountered in front (at a distance that depends on chosen offset).  
        (2) If the chosen side = left, make a right turn at this wall and move forward parallel to it at the chosen offset.  If side = right, make a left turn and do the same
        (3) If a blocking wall is encountered in front, turn away from side wall and continue parallel to blocking wall.  
        (4) If the side wall is "lost," turn toward that side until a wall is refound, then continue forward

    """
    def laser_callback(self, las):
        ranges = las.ranges
        left = len(ranges)//4
        right = (len(ranges))//4*3
        back = len(ranges)//2
        left_front = len(ranges)//8
        right_front = (len(ranges))//8*7
        front = -1

        if not self.following:
            if ranges[front] < .5:
                self.following = True
            else:
                forward = Twist()
                forward.linear.x = .1
                self.publisher.publish(forward)                

        else:
            twist = Twist()
            if ranges[front] < 0.4:
                twist.linear.x = .1
                twist.angular.z = 0.05
                self.publisher.publish(twist)
            else:
                twist.linear.x = .1
                right_average = (ranges[right]+ranges[right+2]+ranges[right-2]+ranges[right-3] +ranges[right+3])/5
                if right_average <= 0.4:
                    twist.angular.z = 1.25 * (0.4-right_average)
                self.publisher.publish(twist)
                left_average = (ranges[left]+ranges[left+2]+ranges[left-2]+ranges[left-3] +ranges[left+3])/5
                if left_average <= 0.4:
                    twist.angular.z = -1.25 * (0.4-left_average)
                print(f"front: {ranges[-1]}, left: {ranges[left]}, right: {right_average}") 
                print(f"{self.following}")
            if ranges[front] > 0.4:
                twist.angular.z = 0.0
                self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    laser = Laserfollow()
    rclpy.spin(laser)
    laser.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
