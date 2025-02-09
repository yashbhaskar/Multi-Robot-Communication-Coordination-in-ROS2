#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class LeaderPosePublisher(Node):
    def __init__(self):
        super().__init__('leader_pose_publisher')
        self.subscription = self.create_subscription(Odometry, '/ros2_bot/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Odometry, '/leader_pose', 10)

    def odom_callback(self, msg):
        self.publisher.publish(msg)  # Forward leader's position to /leader_pose

def main():
    rclpy.init()
    node = LeaderPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
