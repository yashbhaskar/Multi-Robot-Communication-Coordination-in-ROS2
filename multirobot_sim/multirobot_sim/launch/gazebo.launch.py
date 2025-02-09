import launch
from launch.substitutions import LaunchConfiguration
import launch_ros
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                             description='Flag to enable use_sim_time and use Gazebo Time'),

        # Launch Gazebo
        launch.actions.ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawn First Robot - master
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'master', '-topic', '/multirobot_sim', '-x', '0', '-y', '0', '-z', '0.1'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Spawn Second Robot - slave
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'slave', '-topic', '/multirobot_sim', '-x', '2', '-y', '0', '-z', '0.1'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])

