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


    """
    
    def laser_callback(self, las):
        if not following:
            
        ranges = las.ranges
        left = len(ranges)//4
        right = (len(ranges))//4*3
        back = len(ranges)//2

        print(f"front: {ranges[-1]}, left: {ranges[left]}, right: {ranges[right]}, back: {ranges[back]}")



def main(args=None):
    rclpy.init(args=args)

    laser = Laserfollow()
    rclpy.spin(laser)
    laser.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
