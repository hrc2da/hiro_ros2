from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
      get_package_share_directory('hiro_dynamixel'),
      'hiro_pid_config.yaml'
      )
    return LaunchDescription([
        # the talker does text to speech on /speech
        Node(
            package='hiro_dynamixel',
            namespace='hiro_dynamixel',
            executable='talker',
            name='talker',
            parameters=[config]
        ),
        # the mover converts cartesian targets to pose commands
        Node(
            package='hiro_dynamixel',
            namespace='hiro_dynamixel',
            executable='mover',
            name='mover',
            parameters=[config]
        ),
        # topview camera
        Node(
            package='hiro_dynamixel',
            namespace='hiro_dynamixel',
            executable='topview',
            name='topview',
            parameters=[config]
        ),
        # swing (base) motor controller (one node per u2d2)
        Node(
            package='hiro_dynamixel',
            namespace='hiro_dynamixel',
            executable='hiro_base',
            parameters=[config]
        ),

        # arm (shoulder and elbow) motor controllers
        Node(
            package='hiro_dynamixel',
            namespace='hiro_dynamixel',
            executable='hiro_arm',
            parameters=[config]
        )
    ])