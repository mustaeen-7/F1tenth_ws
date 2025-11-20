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
    
    # CHANGED: Cartographer config instead of SLAM Toolbox
    cartographer_config_dir = PathJoinSubstitution([f1tenth_description_pkg, 'config'])
    cartographer_config = PathJoinSubstitution([cartographer_config_dir, 'f1tenth_2d.lua'])

    # URDF/Xacro
    xacro_file = PathJoinSubstitution([f1tenth_description_pkg, 'urdf', 'f1tenth.xacro'])
    robot_description = Command([FindExecutable(name='xacro'), ' ', xacro_file])

    return LaunchDescription([
        DeclareLaunchArgument('joy_config', default_value=joy_config),
        DeclareLaunchArgument('vesc_config', default_value=vesc_config),
        DeclareLaunchArgument('sensors_config', default_value=sensors_config),
        DeclareLaunchArgument('mux_config', default_value=mux_config),
        DeclareLaunchArgument('cartographer_config_dir', default_value=cartographer_config_dir),
        DeclareLaunchArgument('cartographer_config', default_value=cartographer_config),

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

        # CARTOGRAPHER NODES (replacing SLAM Toolbox)
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', LaunchConfiguration('cartographer_config_dir'),
                '-configuration_basename', 'f1tenth_2d.lua'
            ],
            remappings=[
                ('scan', 'scan'),  # Make sure this matches your lidar topic
                ('odom', 'odom'),  # Make sure this matches your odometry topic
            ]
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),
    ])