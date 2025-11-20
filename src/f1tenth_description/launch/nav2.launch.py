from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package share directories
    f1tenth_description_pkg = FindPackageShare('f1tenth_description')

    # Config files
    joy_config = PathJoinSubstitution([f1tenth_description_pkg, 'config', 'joy_teleop.yaml'])
    vesc_config = PathJoinSubstitution([f1tenth_description_pkg, 'config', 'vesc.yaml'])
    sensors_config = PathJoinSubstitution([f1tenth_description_pkg, 'config', 'sensors.yaml'])
    mux_config = PathJoinSubstitution([f1tenth_description_pkg, 'config', 'mux.yaml'])
    map_yaml = PathJoinSubstitution([f1tenth_description_pkg, 'maps', 'new_lab_map.yaml'])
    nav2_params = PathJoinSubstitution([f1tenth_description_pkg, 'config', 'nav2_params.yaml'])

    # URDF/Xacro
    xacro_file = PathJoinSubstitution([f1tenth_description_pkg, 'urdf', 'f1tenth.xacro'])
    robot_description = Command([FindExecutable(name='xacro'), ' ', xacro_file])

    return LaunchDescription([
        DeclareLaunchArgument('joy_config', default_value=joy_config),
        DeclareLaunchArgument('vesc_config', default_value=vesc_config),
        DeclareLaunchArgument('sensors_config', default_value=sensors_config),
        DeclareLaunchArgument('mux_config', default_value=mux_config),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            parameters=[LaunchConfiguration('joy_config')]
        ),

        Node(
            package='joy_teleop',
            executable='joy_teleop',
            name='joy_teleop',
            parameters=[LaunchConfiguration('joy_config')]
        ),

        Node(
            package='vesc_ackermann',
            executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc_node',
            parameters=[LaunchConfiguration('vesc_config')]
        ),

        Node(
            package='vesc_ackermann',
            executable='vesc_to_odom_node',
            name='vesc_to_odom_node',
            parameters=[LaunchConfiguration('vesc_config')]
        ),

        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            parameters=[LaunchConfiguration('vesc_config')]
        ),

        Node(
            package='urg_node',
            executable='urg_node_driver',
            name='urg_node',
            parameters=[LaunchConfiguration('sensors_config')]
        ),

        Node(
            package='ackermann_mux',
            executable='ackermann_mux',
            name='ackermann_mux',
            parameters=[LaunchConfiguration('mux_config')],
            remappings=[('ackermann_cmd_out', 'ackermann_drive')]
        ),

        # 🔧 CONTROL BRIDGE - CRITICAL FOR REVERSING
        Node(
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
        ),

        # ➕ NAV2 STARTS HERE
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_params, {'yaml_filename': map_yaml}]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params]
        ),

        # 🆕 WAYPOINT FOLLOWER - NEEDED FOR COMPLETE NAV2
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params]
        ),

        Node(
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
                    'waypoint_follower'  # 🆕 ADDED
                ]
            }]
        )
    ])