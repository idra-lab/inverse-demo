from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# from easy_handeye2.common_launch import arg_calibration_type, arg_tracking_base_frame, arg_tracking_marker_frame, arg_robot_base_frame, \
#     arg_robot_effector_frame


def generate_launch_description():
    print("Calibrating camera 1: SN 20788008")
    # arg_name = DeclareLaunchArgument('name')
    print("Calibrating 20788008")
    node_dummy_calib_eih = Node(package='tf2_ros', executable='static_transform_publisher', name='dummy_publisher',
                                arguments=f'--x 0 --y 0 --z 0.1 --qx 0 --qy 0 --qz 0 --qw 1'.split(' ') + ['--frame-id', "lbr_link_0",
                                                                                                        '--child-frame-id', "lbr_link_ee"])
    
    calib_node = Node(package='easy_handeye2', executable='aruco_tracker', name='aruco_tracker',
                        parameters=[{
                            "image_topic": "camera_raw_1",
                            "marker_id": 2,
                            # "marker_length": 0.09968, #0.27,
                            "marker_length": 0.13,
                            "camera_frame": "cam_1",
                            "marker_frame": "aruco_marker_frame",
                            # SN 20788008
                            "fx": 1058.674316,
                            "fy": 1058.674316,
                            "cx": 1112.441162,
                            "cy": 624.768311,

                            # taken from /usr/local/zed/config/SNxxx.conf
                            # "fx": 1066.17,
                            # "fy": 1066.72,
                            # "cx": 1097.88,
                            # "cy": 613.349,
                            # "k1": -0.0476521,
                            # "k2": 0.0212008,
                            # "p1": -4.48125e-05,
                            # "p2": -0.000737505,
                            # "k3": -0.00895334,

                        }])

    handeye_server = Node(package='easy_handeye2', executable='handeye_server', name='handeye_server', parameters=[{
        'name': "calibrator_cam_1",
        'calibration_type': "eye_on_base",
        'tracking_base_frame': "cam_1",
        'tracking_marker_frame': "aruco_marker_frame",
        'robot_base_frame': "lbr_link_0",
        'robot_effector_frame': "lbr_link_ee"
    }])

    handeye_rqt_calibrator = Node(package='easy_handeye2', executable='rqt_calibrator.py',
                                  name='handeye_rqt_calibrator',
                                  # arguments=['--ros-args', '--log-level', 'debug'],
                                  parameters=[{
                                    'name': "calibrator_cam_1",
                                    'calibration_type': "eye_on_base",
                                    'tracking_base_frame': "cam_1",
                                    'tracking_marker_frame': "aruco_marker_frame",
                                    'robot_base_frame': "lbr_link_0",
                                    'robot_effector_frame': "lbr_link_ee"
                                  }])
    


    rviz_node = Node(package='rviz2', executable='rviz2', name='rviz2',
                     arguments=['-d', get_package_share_directory('easy_handeye2') + '/rviz/rviz_calib.rviz'],
    )

    return LaunchDescription([
        # arg_name,
        # arg_calibration_type,
        # arg_tracking_base_frame,
        # arg_tracking_marker_frame,
        # arg_robot_base_frame,
        # arg_robot_effector_frame,
        
        calib_node,
        
        # node_dummy_calib_eih,
        handeye_server,
        handeye_rqt_calibrator,
        rviz_node,

    ])
