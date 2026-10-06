from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ------------------------------------------------------------------ #
    #  Launch arguments — override from CLI or a parent launch file
    # ------------------------------------------------------------------ #
    cam_serial_arg = DeclareLaunchArgument(
        "cam1_serial_number",
        default_value="38787582",
        description="Serial number of the first ZED camera (0 = first available)",
    )
    cam_frame_arg = DeclareLaunchArgument(
        "cam1_frame_id",
        default_value="zed_camera_1_frame",
        description="TF frame id for the first camera",
    )

    # ------------------------------------------------------------------ #
    #  Camera nodes
    # ------------------------------------------------------------------ #

    camera_node = Node(
        package="smpl_ros",
        executable="zed_smpl_tracking",
        name="smpl_camera_1_node",
        namespace="camera_1",
        output="screen",
        parameters=[
            {
                "serial_number": LaunchConfiguration("cam1_serial_number"),
                "frame_id": LaunchConfiguration("cam1_frame_id"),
            }
        ],
        remappings=[
            ("/smpl_params",          "/camera_1/smpl_params"),
            ("/zed/image",            "/camera_1/zed/image"),
            ("/zed/image/compressed", "/camera_1/zed/image/compressed"),
            ("/zed/depth",            "/camera_1/zed/depth"),
        ],
    )

    # ------------------------------------------------------------------ #
    #  Depth compressedDepth republishers
    # ------------------------------------------------------------------ #

    depth_republish_cam = Node(
        package="image_transport",
        executable="republish",
        name="depth_republish_cam1",
        arguments=["raw", "compressedDepth"],
        remappings=[
            ("in",                  "/camera_1/zed/depth"),
            ("out/compressedDepth", "/camera_1/zed/depth/compressedDepth"),
        ],
        output="screen",
    )

    aruco_node = Node(
        package="smpl_ros",
        executable="aruco_static_tf.py",
        name="aruco_static_tf",
        output="screen",
    )

    return LaunchDescription(
        [
            cam_serial_arg,
            cam_frame_arg,
            camera_node,
            depth_republish_cam,
            aruco_node,
        ]
    )
