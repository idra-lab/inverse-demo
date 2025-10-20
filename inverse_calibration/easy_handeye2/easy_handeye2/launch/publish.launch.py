from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arg_name = DeclareLaunchArgument('name')

    handeye_publisher_1 = Node(package='easy_handeye2', executable='handeye_publisher', name='handeye_publisher', parameters=[{
        'name': "calibrator_cam_1",
    }])

    handeye_publisher_2 = Node(package='easy_handeye2', executable='handeye_publisher', name='handeye_publisher', parameters=[{
        'name': "calibrator_cam_2",
    }])

    return LaunchDescription([
        # arg_name,
        handeye_publisher_1,
        handeye_publisher_2
    ])
