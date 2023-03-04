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
        self.back_timer = None
        self.turn_timer = None

        self.timer_stopper = None

    def back_callback(self):
        go_back = Twist()
        go_back.linear.x = -0.01
        self.hazard_publisher.publish(go_back)
        print("I backed up")

    def turn_callback(self):
        turn = Twist()
        turn.angular.z = 1.0
        self.hazard_publisher.publish(turn)
        print("I'm turning")

    def destroy_back(self):
        if(self.back_timer):
            self.back_timer.destroy()
        if(self.timer_stopper):
            self.timer_stopper.destroy()
        self.hazard_publisher.publish(Twist())
        print("I stopped backing up")

    def destroy_turn(self):
        if(self.turn_timer):
            self.turn_timer.destroy()
        if(self.timer_stopper):
             self.timer_stopper.destroy()
        self.hazard_publisher.publish(Twist())
        print("I stopped turning")

    def hazard_callback(self, haz):
        if len(haz.detections) > 0:
            for i in range(0, len(haz.detections)):
                if(haz.detections[i].type == 1):
                    self.get_logger().info('hazard type!: "%i"' % haz.detections[i].type)
                    #print("bumped")
                    self.hazard_publisher.publish(Twist())
                    #print("stopped")
                    self.back_timer = self.create_timer(.1, self.back_callback)
                    self.timer_stopper = self.create_timer(1, self.destroy_back)
                    time.sleep(1.5)
                    print("backing up")
                    turn_time = random.uniform(2,4)
                    self.turn_timer = self.create_timer(.1, self.turn_callback)
                    self.timer_stopper = self.create_timer(turn_time, self.destroy_turn)
                    print("turning")

    def tester(self):
#        self.get_logger().info('hazard type!: "%i"' % haz.detections[i].type)
        #print("bumped")
        self.hazard_publisher.publish(Twist())
        #print("stopped")
        self.back_timer = self.create_timer(.1, self.back_callback)
        self.timer_stopper = self.create_timer(1, self.destroy_back)
        time.sleep(1.5)
        print("backing up")
        turn_time = random.uniform(2,4)
        self.turn_timer = self.create_timer(.1, self.turn_callback)
        self.timer_stopper = self.create_timer(turn_time, self.destroy_turn)
        print("turning")

#ros2 topic pub qbert/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
def main(args=None):
    rclpy.init(args=args)

    wanderer = Wanderer()
    
    rclpy.spin(wanderer)
    wanderer.tester()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wanderer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



