#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class Follower(Node):
    def __init__(self):
        super().__init__('follower')
        self.subscription = self.create_subscription(Odometry, '/leader_pose', self.pose_callback, 10)
        self.publisher = self.create_publisher(Twist, '/tortoisebot/cmd_vel', 10)
        self.target_distance = 1.0  # Maintain 1m distance from leader

    def pose_callback(self, msg):
        leader_x = msg.pose.pose.position.x
        leader_y = msg.pose.pose.position.y

        # Set a target behind leader
        target_x = leader_x - self.target_distance
        target_y = leader_y

        # Compute movement
        twist = Twist()
        distance = math.sqrt((target_x - leader_x) ** 2 + (target_y - leader_y) ** 2)

        if distance > self.target_distance:
            twist.linear.x = 0.5  # Move forward
        else:
            twist.linear.x = 0.0  # Stop
        
        self.publisher.publish(twist)

def main():
    rclpy.init()
    node = Follower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
