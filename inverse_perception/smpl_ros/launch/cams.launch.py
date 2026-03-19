from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ------------------------------------------------------------------ #
    #  Launch arguments — override from CLI or a parent launch file
    # ------------------------------------------------------------------ #

    # Camera 1
    cam1_serial_arg = DeclareLaunchArgument(
        "cam1_serial_number",
        default_value="38787582",
        description="Serial number of the first ZED camera (0 = first available)",
    )
    cam1_frame_arg = DeclareLaunchArgument(
        "cam1_frame_id",
        default_value="zed_camera_1_frame",
        description="TF frame id for the first camera",
    )

    # Camera 2
    cam2_serial_arg = DeclareLaunchArgument(
        "cam2_serial_number",
        default_value="27662210",
        description="Serial number of the second ZED camera (0 = first available)",
    )
    cam2_frame_arg = DeclareLaunchArgument(
        "cam2_frame_id",
        default_value="zed_camera_2_frame",
        description="TF frame id for the second camera",
    )

    # ------------------------------------------------------------------ #
    #  Camera nodes
    # ------------------------------------------------------------------ #

    camera_1_node = Node(
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

    camera_2_node = Node(
        package="smpl_ros",
        executable="zed_smpl_tracking",
        name="smpl_camera_2_node",
        namespace="camera_2",
        output="screen",
        parameters=[
            {
                "serial_number": LaunchConfiguration("cam2_serial_number"),
                "frame_id": LaunchConfiguration("cam2_frame_id"),
            }
        ],
        remappings=[
            ("/smpl_params",          "/camera_2/smpl_params"),
            ("/zed/image",            "/camera_2/zed/image"),
            ("/zed/image/compressed", "/camera_2/zed/image/compressed"),
            ("/zed/depth",            "/camera_2/zed/depth"),
        ],
    )

    # ------------------------------------------------------------------ #
    #  Depth compressedDepth republishers
    # ------------------------------------------------------------------ #

    depth_republish_cam1 = Node(
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

    depth_republish_cam2 = Node(
        package="image_transport",
        executable="republish",
        name="depth_republish_cam2",
        arguments=["raw", "compressedDepth"],
        remappings=[
            ("in",                  "/camera_2/zed/depth"),
            ("out/compressedDepth", "/camera_2/zed/depth/compressedDepth"),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            # Arguments
            cam1_serial_arg,
            cam1_frame_arg,
            cam2_serial_arg,
            cam2_frame_arg,
            # Camera nodes
            camera_1_node,
            camera_2_node,
            # Depth republishers
            depth_republish_cam1,
            depth_republish_cam2,
        ]
    )