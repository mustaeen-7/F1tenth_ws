from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    # Package share directories
    f1tenth_description_pkg = FindPackageShare('f1tenth_description')
    f1tenth_hardware_pkg = FindPackageShare('f1tenth_hardware')

    # Config files
    joy_config = PathJoinSubstitution([f1tenth_description_pkg, 'config', 'joy_teleop.yaml'])
    vesc_config = PathJoinSubstitution([f1tenth_description_pkg, 'config', 'vesc.yaml'])
    sensors_config = PathJoinSubstitution([f1tenth_description_pkg, 'config', 'sensors.yaml'])
    mux_config = PathJoinSubstitution([f1tenth_description_pkg, 'config', 'mux.yaml'])
    ekf_config = PathJoinSubstitution([f1tenth_description_pkg, 'config', 'sensor_fusion.yaml'])
    map_file = PathJoinSubstitution([f1tenth_description_pkg, 'maps', 'full_map.yaml'])
    
    # New AMCL config
    amcl_config = PathJoinSubstitution([f1tenth_description_pkg, 'config', 'amcl.yaml'])

    # URDF/Xacro
    xacro_file = PathJoinSubstitution([f1tenth_hardware_pkg, 'urdf', 'f1tenth_robot.urdf.xacro'])
    robot_description = Command([FindExecutable(name='xacro'), ' ', xacro_file])

    return LaunchDescription([

        # Declare launch arguments
        DeclareLaunchArgument('joy_config', default_value=joy_config),
        DeclareLaunchArgument('vesc_config', default_value=vesc_config),
        DeclareLaunchArgument('sensors_config', default_value=sensors_config),
        DeclareLaunchArgument('mux_config', default_value=mux_config),
        DeclareLaunchArgument('ekf_config', default_value=ekf_config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        # AMCL (map-based localization) arguments
        DeclareLaunchArgument('use_amcl', default_value='true', 
                             description='Use AMCL for map-based localization'),
        DeclareLaunchArgument('map_file', default_value='', 
                             description='Path to map yaml file for AMCL'),
        DeclareLaunchArgument('amcl_config', default_value=amcl_config),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        # Joint state publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # Static transform: base_link to laser (adjust these values based on your mounting)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0.27', '0', '0.11', '0', '0', '0', 'base_link', 'laser']
        ),

        # Static transform: base_link to camera (adjust these values based on your mounting)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=['0.3', '0', '0.15', '0', '0', '0', 'base_link', 'camera_link']
        ),

        # Joystick nodes
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

        # VESC nodes
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
            parameters=[LaunchConfiguration('vesc_config')],
        ),

        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            parameters=[LaunchConfiguration('vesc_config')]
        ),

        # Lidar node (Hokuyo)
        Node(
            package='urg_node',
            executable='urg_node_driver',
            name='urg_node',
            parameters=[LaunchConfiguration('sensors_config')],
        ),

        # RealSense Camera Node
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera_node',
            output='screen',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'enable_gyro': True,
                'enable_accel': True,
                'gyro_fps': 200,
                'accel_fps': 200,
                'enable_sync': True,
                'unite_imu_method': 2,  # We'll combine manually
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            remappings=[
                ('/camera/color/image_raw', '/camera/color/image_raw'),
                ('/camera/depth/image_raw', '/camera/depth/image_raw'),
                ('/camera/depth/color/points', '/camera/depth/color/points')
            ]
        ),

        # # IMU Combiner Node (combines RealSense accel and gyro)
        # Node(
        #     package='f1tenth_description',  # Your package name
        #     executable='imu_bridge',
        #     name='imu_bridge',
        #     output='screen',
        #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        # ),

        # ===== AMCL MAP-BASED LOCALIZATION =====
        
        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': LaunchConfiguration('map_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            condition=IfCondition(LaunchConfiguration('use_amcl'))
        ),

        # AMCL Node (map-based localization using LiDAR)
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[LaunchConfiguration('amcl_config')],
            condition=IfCondition(LaunchConfiguration('use_amcl')),
            remappings=[
                ('scan', '/scan'),
                ('initialpose', '/initialpose'),
                ('amcl_pose', '/amcl_pose')
            ]
        ),

        # Lifecycle Manager (for Nav2 nodes)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }],
            condition=IfCondition(LaunchConfiguration('use_amcl'))
        ),

        # EKF for sensor fusion (robot_localization) - With AMCL pose fusion
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[LaunchConfiguration('ekf_config')],
            remappings=[
                ('odometry/filtered', 'odometry/filtered'),
                ('/set_pose', '/initialpose')
            ]
        ),

        # Ackermann mux node
        Node(
            package='ackermann_mux',
            executable='ackermann_mux',
            name='ackermann_mux',
            parameters=[LaunchConfiguration('mux_config')],
            remappings=[('ackermann_cmd_out', 'ackermann_drive')]
        ),

    ])