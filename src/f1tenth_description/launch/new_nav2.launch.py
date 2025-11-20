#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    f1tenth_description_pkg = FindPackageShare('f1tenth_description')
    sensors_config = PathJoinSubstitution([f1tenth_description_pkg, 'config', 'sensors.yaml'])
    joy_config = PathJoinSubstitution([f1tenth_description_pkg, 'config', 'joy_teleop.yaml'])
    ekf_config = PathJoinSubstitution([f1tenth_description_pkg, 'config', 'sensor_fusion.yaml'])
    mux_config = PathJoinSubstitution(
        [
            FindPackageShare("f1tenth_description"),
            "config",
            "mux.yaml"
        ]
    )
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use sim time if true",
        )
    )
    declared_arguments.append(
         DeclareLaunchArgument('sensors_config', default_value=sensors_config),
    )
    declared_arguments.append(
        DeclareLaunchArgument('joy_config', default_value=joy_config),
    )
    declared_arguments.append(
        DeclareLaunchArgument('mux_config', default_value=mux_config),
    )

    # Initialize arguments
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("f1tenth_hardware"),
                    "urdf",
                    "f1tenth_robot.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Controller configuration (only joint state broadcaster)
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("f1tenth_hardware"),
            "config",
            "f1tenth_controllers.yaml",
        ]
    )

    # Control node (for state publishing only)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # Robot state publisher (critical for nav2 TF tree)
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Joint state broadcaster spawner (publishes joint states for nav2)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # ==================== VESC CONTROL SYSTEM ====================
    # These handle actual motor control - nav2 sends commands here
    
    Joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy",
        parameters=[LaunchConfiguration('joy_config')]
    )
    
    Joy_teleop_node = Node(
        package="joy_teleop",
        executable="joy_teleop",
        name="joy_teleop",
        parameters=[LaunchConfiguration('joy_config')]
    )
    
    # Ackermann to VESC node (receives /cmd_vel from nav2)
    ackermann_to_vesc_node = Node(
        package="vesc_ackermann", 
        executable="ackermann_to_vesc_node",
        name="ackermann_to_vesc_node",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("f1tenth_hardware"),
                "config",
                "vesc_config.yaml"
            ]),
            {"use_sim_time": use_sim_time}
        ]
    )

    # VESC to Odom node (publishes /odom topic for nav2)
    vesc_to_odom_node = Node(
        package="vesc_ackermann",
        executable="vesc_to_odom_node",
        name="vesc_to_odom_node",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("f1tenth_hardware"),
                "config",
                "vesc_config.yaml" 
            ]),
            {"use_sim_time": use_sim_time}
        ]
    )
    
    # VESC driver node
    vesc_driver_node = Node(
        package="vesc_driver",
        executable="vesc_driver_node",
        name="vesc_driver_node",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("f1tenth_hardware"),
                "config", 
                "vesc_config.yaml"
            ]),
            {"use_sim_time": use_sim_time}
        ]
    )
    
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_cmd_out', 'ackermann_drive')]
    )

    # ==================== SENSORS ====================
    
    # Hokuyo LiDAR driver
    hokuyo_node = Node(
        package="urg_node",
        executable="urg_node_driver",
        name="urg_node",
        parameters=[LaunchConfiguration('sensors_config'), {"use_sim_time": use_sim_time}]
    )

    # RealSense camera node (optional for nav2)
    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="realsense2_camera",
        parameters=[{
            "enable_color": True,
            "enable_depth": True,
            "enable_infra1": True,
            "enable_infra2": True,
            "color_width": 640,
            "color_height": 480,
            "depth_width": 640, 
            "depth_height": 480,
            "color_fps": 30.0,
            "depth_fps": 30.0,
            "use_sim_time": use_sim_time,
        }],
        condition=IfCondition("true")  # Set to false to disable camera
    )
    
    control_bridge_node = Node(
        package='f1tenth_description',  # Replace with your actual package name
        executable='control_bridge',   # Your compiled control bridge executable
        name='control_bridge',
        output='screen',
        parameters=[{
            'twist_topic': '/cmd_vel',
            'ackermann_topic': '/drive',
            'wheelbase': 0.25,
            'update_rate': 50.0,
            'max_steering_angle': 0.5236  # 30 degrees
        }]
    )

    # ==================== NAV2 CONFIGURATION ====================
    nav2_params = PathJoinSubstitution([f1tenth_description_pkg, 'config', 'nav2_params.yaml'])
    map_yaml = PathJoinSubstitution([f1tenth_description_pkg, 'maps', 'full_map.yaml'])

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params, {'yaml_filename': map_yaml}]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params]
    )

    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params]
    )

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params]
    )
    
    behavior_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params]
    )
    
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params]
    )

    # Waypoint follower - needed for complete nav2
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params]
    )
    
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower'
            ]
        }]
    )
    ekf_node=Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_config]
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        ackermann_mux_node,
        vesc_driver_node,
        vesc_to_odom_node, 
        ackermann_to_vesc_node,
        Joy_node,
        Joy_teleop_node,
        hokuyo_node,
        realsense_node,
        control_bridge_node,
        map_server_node,
        amcl_node,
        planner_node,
        controller_node,
        behavior_node,
        bt_navigator_node,
        waypoint_follower_node,
        lifecycle_manager_node,
        ekf_node
    ]

    return LaunchDescription(declared_arguments + nodes)