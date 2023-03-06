import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from irobot_create_msgs.msg import HazardDetectionVector
    
from geometry_msgs.msg import Twist
from rclpy import qos
import random
import time

class Wanderer(Node):

    def __init__(self):
        super().__init__('wanderer')
        
        self.hazard_subscription = self.create_subscription(
            HazardDetectionVector,
            'qbert/hazard_detection',
            self.hazard_callback,
            qos.qos_profile_sensor_data)

        self.hazard_subscription  # prevent unused variable warning
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

    # each callback function gets called by our timers at a rate specified in {create_timer}
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

    def hazard_callback(self, haz):
        if len(haz.detections) > 0:
            types = [val.type for val in haz.detections] #list comprehension extracts types from list of objects
            if (1 in types) and not self.executing: #type 1 is collision
                if(self.forward_timer):
                    self.destroy_timer(self.forward_timer) #stop going forward!!!
                self.executing = True    
                print("bumped")
                self.hazard_publisher.publish(Twist())
                self.back_timer = self.create_timer(.1, self.back_callback)
                self.timer_stopper = self.create_timer(2, self.destroy_back)
                print("backing up")
                self.delayer_a = self.create_timer(2, self.delayed_turn)
                self.delayer_b = self.create_timer(7, self.delayed_forward)


#ros2 topic pub qbert/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
def main(args=None):
    rclpy.init(args=args)

    wanderer = Wanderer()
    
    rclpy.spin(wanderer)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wanderer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



