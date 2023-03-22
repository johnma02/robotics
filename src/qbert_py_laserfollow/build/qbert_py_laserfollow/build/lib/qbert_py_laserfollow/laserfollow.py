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

    def hazard_callback(self, haz):
        pass
    
    def laser_callback(self, las):
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
