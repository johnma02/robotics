import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from irobot_create_msgs.msg import HazardDetectionVector
from sensor_msgs.msg import LaserScan
    
from geometry_msgs.msg import Twist
from rclpy import qos
import random
import time

class Laserfollow:

    def __init__(self):
        super().__init__()

        self.hazard_subscription = self.create_subscription(
                HazardDetectionVector,
                'qbert/hazard_detection',
                self.hazard_callback,
                qos.qos_profile_sensor_data)
        
        self.subscription = self.rospy.Subscriber('/qbert/laser_scan', LaserScan, callback)
        self.subscription  # prevent unused variable warning

        self.hazard_publisher = self.create_publisher(Twist, "qbert/cmd_vel",10)
        # timer for each movement type
        self.back_timer = None
        self.turn_timer = None
        self.forward_timer = None
        # timer to stop timers
        self.timer_stopper = None
        # boolean ensures only one process runs at a time
        self.executing = False
        # need two delay timers for our algorithm
        self.delayer_a = None
        self.delayer_b = None

    def back_callback(self):
        go_back = Twist()
        go_back.linear.x = -0.01
        self.hazard_publisher.publish(go_back)
        print(f'I\'m backing up at {go_back.linear.x}m/s')

    def forward_callback(self):
        forward = Twist()
        forward.linear.x = .1
        self.hazard_publisher.publish(forward)
        print(f'I\'m moving forward at {forward.linear.x}m/s')

    def turn_callback(self):
        turn = Twist()
        turn.angular.z = 1.0
        self.hazard_publisher.publish(turn)
        print(f'I\'m turning at {turn.angular.z}rad/s')

    # timer destroyers ensure our timers do not run forever
    def destroy_back(self):
        self.destroy_timer(self.back_timer)
        self.destroy_timer(self.timer_stopper)
        self.hazard_publisher.publish(Twist())
        print("I stopped backing up")

    def destroy_turn(self):
        self.destroy_timer(self.turn_timer)
        self.destroy_timer(self.timer_stopper)
        self.hazard_publisher.publish(Twist())
        print("I stopped turning")

    def delayed_turn(self):
        turn_time = random.uniform(4,5)
        self.turn_timer = self.create_timer(.1, self.turn_callback)
        self.timer_stopper = self.create_timer(turn_time, self.destroy_turn)
        self.destroy_timer(self.delayer_a)

    def delayed_forward(self):
        self.executing = False #{delayed_forward} is the last component in our wandering algorithm, so we indicate we are finished executing
        self.forward_timer = self.create_timer(.1, self.forward_callback)
        self.destroy_timer(self.delayer_b)


    def move(self, haz):
        while 1:
            print(self.subscription.ranges)





def main(args=None):
    rclpy.init(args=args)

    laser = Laserfollow()
    rclpy.spin(laser)
    laser.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
