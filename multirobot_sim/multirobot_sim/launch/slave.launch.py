import os
import launch
import launch_ros.actions
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess

def generate_launch_description():
    xacro_file = "/home/yash/ros_ws/src/multirobot_sim/slave_models/urdf/tortoisebot.xacro"
    urdf_file = "/home/yash/ros_ws/src/multirobot_sim/slave_models/urdf/tortoisebot.urdf"

    return launch.LaunchDescription([
        # Convert Xacro to URDF
        ExecuteProcess(
            cmd=[FindExecutable(name="xacro"), xacro_file, "-o", urdf_file],
            output="screen"
        ),
        
        # Spawn the robot in Gazebo with a distance from ros2_bot
        launch_ros.actions.Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-entity", "tortoisebot", "-file", urdf_file, "-x", "2.0", "-y", "2.0", "-z", "0.1"],
            output="screen"
        ),

        # Launch the follower script for leader-follower behavior
        launch_ros.actions.Node(
            package="multirobot_sim",
            executable="follower.py",
            name="tortoisebot_follower",
            output="screen"
        )
    ])
