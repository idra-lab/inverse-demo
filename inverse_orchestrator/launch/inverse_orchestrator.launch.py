from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import datetime


def launch_setup(context, *args, **kwargs):

    # ── GET MODE (resolved) ─────────────────────────────────────
    mode = LaunchConfiguration("mode").perform(context)

    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

    bag_name = f"/media/ictadmin/LeoSSD/IntegrationWeek/recordings/rosbag_{mode}_{timestamp}"

    # ── LOG ─────────────────────────────────────────────────────
    log = LogInfo(msg=f"Recording rosbag: {bag_name}")

    topics = [
        # ── Cameras ──────────────────────────────────────────────────────────
        "/camera/realsense2_camera/color/image_raw/compressed",
        "/camera/realsense2_camera/color/camera_info",
        "/camera/realsense2_camera/aligned_depth_to_color/image_raw/compressedDepth",
        # "/zed/image", only for single cam

        "/camera_1/zed/depth/compressedDepth",
        "/camera_1/zed/image/compressed",
        "/camera_1/zed/camera_info",

        "/camera_2/zed/depth/compressedDepth",
        "/camera_2/zed/image/compressed",
        "/camera_2/zed/camera_info",
        
        # ── Joint states ─────────────────────────────────────────────────────
        "/joint_states",
        "/dynamic_joint_states",

        # ── Gripper width & grasping forces ──────────────────────────────────
        "/robotiq_gripper_trajectory_controller/state",
        "/io_and_status_controller/tool_data",

        # ── Cartesian EEF pose & twist ───────────────────────────────────────
        "/cartesian_motion_controller/current_pose",
        "/cartesian_motion_controller/current_twist",

        # ── Commanded poses (controller + gripper) ───────────────────────────
        "/target_frame",
        "/joint_trajectory_controller/joint_trajectory",
        "/robotiq_gripper_trajectory_controller/joint_trajectory",

        # ── TF & camera infos ────────────────────────────────────────────────
        "/target_link_pose", # final link current measured pose 
        "/tf",
        "/tf_static",

        # SMPL
        "/camera_1/smpl_params",
        "/camera_2/smpl_params",

    ]
    rosbag_record = ExecuteProcess(
        cmd=["ros2", "bag", "record", "--output", bag_name] + topics,
        output="screen",
    )

    # ── ORCHESTRATOR ────────────────────────────────────────────
    orchestrator = Node(
        prefix="xterm -fa Monospace -fs 16 -e",
        package="inverse_orchestrator",
        executable="orchestrator",
        name="orchestrator",
        output="screen",
        parameters=[{"mode": mode}]
    )

    tf_pose_publisher = Node(
        package="inverse_orchestrator",
        executable="tf_publisher",
        name="tf_publisher",
        output="screen",
    )

    return [log, tf_pose_publisher, rosbag_record, orchestrator]


def generate_launch_description():

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="forward",
        description="Execution mode: forward or inverse"
    )

    return LaunchDescription([
        mode_arg,
        OpaqueFunction(function=launch_setup)
    ])