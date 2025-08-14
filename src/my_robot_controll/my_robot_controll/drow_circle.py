#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Drow_Circle_Node(Node):
    def __init__(self):
        super().__init__('drow_circle')
        self.cmd_val_pub_ = self.create_publisher(Twist,'/turtle1/cmd_vel',10)
        self.timer_ = self.create_timer(0.5,self.SendCercleCmd)
        self.get_logger().info("Drow Circle")

    def SendCercleCmd(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.cmd_val_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Drow_Circle_Node()
    rclpy.spin(node)
    rclpy.shutdown()