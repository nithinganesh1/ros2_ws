#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class Clossed_Loop(Node):
    def __init__(self):
        super().__init__('edge_avoiding_turtle')
        self.get_logger().info("Started turtle contorl")
        self.publisher_ = self.create_publisher(Twist,'/turtle1/cmd_vel',10)
        self.pose_subscriber_ = self.create_subscription(Pose,'/turtle1/pose',self.pose_callback,10)
    def pose_callback(self, msg: Pose):
        cmd = Twist()
        if msg.x > 9 or msg.x < 2 or msg.y > 9 or msg.y <2:
            cmd.linear.x = 1.0
            cmd.angular.z = 0.9
        else:
            cmd.linear.x = 5.0
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Clossed_Loop()
    rclpy.spin(node)
    rclpy.shutdown()