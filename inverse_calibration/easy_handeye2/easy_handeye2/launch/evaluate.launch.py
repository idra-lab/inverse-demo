from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    arg_name = DeclareLaunchArgument('name')

    handeye_rqt_evaluator = Node(package='easy_handeye2', executable='rqt_evaluator.py',
                                  name='handeye_rqt_evaluator',
                                  # arguments=['--ros-args', '--log-level', 'debug'],
                                  parameters=[{
                                      'name': LaunchConfiguration('name'),
                                  }])
    cameras_tf_publisher = IncludeLaunchDescription(
         launch_description_source = get_package_share_directory("easy_handeye2") + "/launch/publish.launch.py"
    )
    rviz_node = Node(package='rviz2', executable='rviz2', name='rviz2',
                     arguments=['-d', get_package_share_directory('easy_handeye2') + '/rviz/rviz_calib.rviz'],
    )
    return LaunchDescription([
        # arg_name,
        # handeye_rqt_evaluator,
        cameras_tf_publisher,
        rviz_node
    ])
