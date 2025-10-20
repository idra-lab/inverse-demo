import os
import re
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
    LogInfo,
    Shutdown
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def robot_gripper_spawner(
    context: LaunchContext,
    robot_ip, 
    robot_name, # TODO: not sure this is conventional
    use_fake_hardware,
    arm_id = 'fr3'
):

    robot_ip_str = context.perform_substitution(robot_ip)

    default_joint_name_postfix = '_finger_joint'
    joint_names_1 = robot_name + '_' + arm_id + default_joint_name_postfix + '1'
    joint_names_2 = robot_name + '_' + arm_id + default_joint_name_postfix + '2'
    joint_names = [joint_names_1, joint_names_2]


    # Load the gripper configuration file
    gripper_config = os.path.join(
        get_package_share_directory('franka_gripper'), 'config', 'franka_gripper_node.yaml'
    )

    return [
        Node(
            package='franka_gripper',
            executable='franka_gripper_node',
            name=['franka_gripper'],
            namespace=robot_name,
            parameters=[
                {
                    'robot_ip': robot_ip_str,
                    'joint_names': joint_names
                }, 
                gripper_config
            ],
            condition=UnlessCondition(use_fake_hardware),
        )
    ]

def robot_description_dependent_nodes_spawner(
        context: LaunchContext,
        xacro_file, 
        use_gazebo,
        use_fake_hardware,
        fake_sensor_commands,
        load_gripper,
        gripper_type,
        self_collisions,
        limit_override,
        left_ip,
        right_ip,
        controller_file,
    ):

    xacro_file_str = context.perform_substitution(xacro_file)
    controller_file_str = context.perform_substitution(controller_file)
    load_gripper_str = context.perform_substitution(load_gripper)
    gripper_type_str = context.perform_substitution(gripper_type)

    use_gazebo_str = context.perform_substitution(use_gazebo)
    self_collisions_str = context.perform_substitution(self_collisions)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    fake_sensor_commands_str = context.perform_substitution(
        fake_sensor_commands
    )

    limit_override_str = context.perform_substitution(limit_override)
    left_ip_str = context.perform_substitution(left_ip)
    right_ip_str = context.perform_substitution(right_ip)

    p = '\t' # Padding
    robot_description = xacro.process_file(
        xacro_file_str,
        mappings = {      
            'hand': load_gripper_str,
            'ee_id': gripper_type_str, 
            
            'gazebo': use_gazebo_str,
            'ros2_control': 'false',
            'with_sc': self_collisions_str,
            'use_fake_hardware': use_fake_hardware_str,
            'fake_sensor_commands': fake_sensor_commands_str,

            'limit_override': limit_override_str,
            'franka1_ip': right_ip_str,
            'franka2_ip': left_ip_str,

            'controller_path': controller_file_str
        }
    ).toprettyxml(p)

    # print(robot_description)

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),        
        
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                controller_file,
                {'load_gripper': load_gripper_str}
            ],
            remappings=[
                ('~/robot_description', '/robot_description'),
                ('joint_states', '/joint_states')
            ],
            output='screen',
            condition=UnlessCondition(use_gazebo),
            on_exit=Shutdown(),
        ),
    ]


def generate_launch_description():
    xacro_file_parameter_name = 'xacro_file'
    controller_file_parameter_name = 'controller_file'
    rviz_file_parameter_name = 'rviz_file'
    load_gripper_parameter_name = 'load_gripper'
    gripper_type_parameter_name = 'gripper_type'

    use_rviz_parameter_name = 'use_rviz'
    use_gazebo_parameter_name = 'use_gazebo'
    enable_self_collisions_parameter_name = 'enable_self_collisions'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'use_fake_sensor_commands'

    limit_override_name = 'limit_override'
    robot_left_ip_parameter_name = 'left_ip'
    robot_right_ip_parameter_name = 'right_ip'

    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    gripper_type = LaunchConfiguration(gripper_type_parameter_name)

    use_rviz = LaunchConfiguration(use_rviz_parameter_name)
    use_gazebo = LaunchConfiguration(use_gazebo_parameter_name)
    enable_self_collisions = LaunchConfiguration(enable_self_collisions_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(
        fake_sensor_commands_parameter_name
    )

    limit_override = LaunchConfiguration(limit_override_name)
    left_ip = LaunchConfiguration(robot_left_ip_parameter_name)
    right_ip = LaunchConfiguration(robot_right_ip_parameter_name)
    
    xacro_file = LaunchConfiguration(xacro_file_parameter_name)
    controller_file = LaunchConfiguration(controller_file_parameter_name)
    rviz_file = LaunchConfiguration(rviz_file_parameter_name)

    joint_state_publisher_sources = ["/joint_states", "franka1/franka_gripper/joint_states", "franka2/franka_gripper/joint_states"]

    robot_description_dependent_nodes_spawner_opaque_function = OpaqueFunction(
        function=robot_description_dependent_nodes_spawner,
        args=[
            xacro_file,
            use_gazebo,
            use_fake_hardware,
            fake_sensor_commands,
            load_gripper,
            gripper_type,
            enable_self_collisions,
            limit_override,
            left_ip,
            right_ip,
            controller_file,
        ]
    )

    robot_gripper_left = OpaqueFunction(
        function=robot_gripper_spawner,
        args=[
            left_ip,
            'franka2',
            use_fake_hardware, 
        ]
    )

    robot_gripper_right = OpaqueFunction(
        function=robot_gripper_spawner,
        args=[
            right_ip,
            'franka1',
            use_fake_hardware, 
        ]
    )
    
    # Gazebo specific configurations
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(get_package_share_directory('franka_description'))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description'],
        output='screen',
        condition=IfCondition(use_gazebo)
    )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    on_shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                LogInfo(msg='Shutting down cleanly...'),
            ]
        )
    )

    launch_description = LaunchDescription([
        DeclareLaunchArgument(
            xacro_file_parameter_name,
            description='Open and spawn the robot in Gazebo',
            default_value=os.path.join(get_package_share_directory("inverse_bringup"), "urdf", "prisma.urdf.xacro"),        
        ),
        DeclareLaunchArgument(
            controller_file_parameter_name,
            description='Controller configuration file',
            default_value=os.path.join(get_package_share_directory("inverse_bringup"), "config", "prisma_controllers.yaml"),
        ),
        DeclareLaunchArgument(
            rviz_file_parameter_name,
            description='Rviz configuration',
            default_value=os.path.join(get_package_share_directory("inverse_bringup"), "rviz", "bimanual.rviz"),
        ),
        DeclareLaunchArgument(
            use_gazebo_parameter_name,
            description='Open and spawn the robot in Gazebo',
            default_value='false'
        ),
        DeclareLaunchArgument(
            use_rviz_parameter_name,
            default_value='true',
            description='Visualize the robot in Rviz'
        ),
        DeclareLaunchArgument(
            enable_self_collisions_parameter_name,
            default_value='false',
            description='Enables self collisions.'
        ),
        DeclareLaunchArgument(
            use_fake_hardware_parameter_name,
            default_value='false',
            description='Use fake hardware'
        ),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter_name,
            default_value='false',
            description='Fake sensor commands. Only valid when "{}" is true'.format(
                use_fake_hardware_parameter_name
            )
        ),
        DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='true',
            description='Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
                        'without an end-effector.'
        ),

        # TODO Cobot pump not working
        DeclareLaunchArgument(
            gripper_type_parameter_name,
            description='Select the gripper type of the end-effector. '
                        'You can select between franka_hand and cobot_pump',
            default_value='franka_hand'
        ),
        DeclareLaunchArgument(
            limit_override_name,
            description='If set to true, hardware interface will skip '
                        'franka::limitRate controls',
            default_value='false'
        ),
        DeclareLaunchArgument(
            robot_left_ip_parameter_name,
            description='IP of the left robot arm. '
                        'This parameter is used only when not in Gazebo',
            default_value='192.168.0.1'
        ),
        DeclareLaunchArgument(
            robot_right_ip_parameter_name,
            description='IP of the right robot arm. '
                        'This parameter is used only when not in Gazebo',
            default_value='192.168.0.2'
        ),

        robot_description_dependent_nodes_spawner_opaque_function,
        
        # WARN: Gripper works but introduces delays in controls
        robot_gripper_left,
        robot_gripper_right,

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'source_list': joint_state_publisher_sources,
                'rate': 1000.0,
                'use_robot_description': False,
            }],
            output='screen',
        ),

        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': 'empty.sdf -r --render-engine ogre'}.items(),
            condition=IfCondition(use_gazebo)
        ),

        # Launch Rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['--display-config', rviz_file],
            condition=IfCondition(use_rviz)
        ),

        # Spawn
        spawn,

        # Controllers
        TimerAction(
            period=2.0,
            actions=[
                load_joint_state_broadcaster, 
            ]
        ),
        on_shutdown
    ])

    return launch_description
