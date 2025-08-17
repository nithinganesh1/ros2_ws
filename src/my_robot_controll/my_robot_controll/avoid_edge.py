#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial

class Clossed_Loop(Node):
    def __init__(self):
        self.previous_x_ = 0
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
        
        if msg.x >= 5.5 and self.previous_x_ <= 5.5:
            self.previous_x_= msg.x
            self.get_logger().info('Changed Color to Red')
            self.call_set_pen_sev(255,0,0,5,0)
        elif msg.x <= 5.5 and self.previous_x_ >= 5.5:
            self.previous_x_ = msg.x
            self.get_logger().info("Changed Color to Green")
            self.call_set_pen_sev(0,255,0,5,0)

        self.publisher_.publish(cmd)
    
    def call_set_pen_sev(self,r,g,b,width,off): 
        clinet = self.create_client(SetPen,'/turtle1/set_pen')
        while not clinet.wait_for_service(1.0): # wait for 1 second for service is available or not
            self.get_logger().warn("Waiting for the response")

        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = clinet.call_async(request=request)
        future.add_done_callback(partial(self.callback_set_pen))

    def callback_set_pen(self,future): # for handling request
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service called faild {e}")
        

def main(args=None):
    rclpy.init(args=args)
    node = Clossed_Loop()
    rclpy.spin(node)
    rclpy.shutdown()