import os
import launch
import launch_ros.actions
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    xacro_file = "/home/yash/ros_ws/src/multirobot_sim/master_models/urdf/ros2_bot.xacro"
    urdf_file = "/home/yash/ros_ws/src/multirobot_sim/master_models/urdf/ros2_bot.urdf"

    return launch.LaunchDescription([
        # Convert Xacro to URDF
        ExecuteProcess(
            cmd=[FindExecutable(name="xacro"), xacro_file, "-o", urdf_file],
            output="screen"
        ),
        
        # Spawn the robot in Gazebo
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-entity", "ros2_bot", "-file", urdf_file, "-x", "0.0", "-y", "0.0", "-z", "0.1"],
            output="screen"
        ),
        
        # Launch leader_pose_publisher
        Node(
            package="multirobot_sim",
            executable="leader.py",
            output="screen"
        )
    ])
