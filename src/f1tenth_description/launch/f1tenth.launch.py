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

    # URDF/Xacro
    xacro_file = PathJoinSubstitution([f1tenth_description_pkg, 'urdf', 'f1tenth.xacro'])
    robot_description = Command([FindExecutable(name='xacro'), ' ', xacro_file])

    return LaunchDescription([

        # Declare launch arguments
        DeclareLaunchArgument('joy_config', default_value=joy_config),
        DeclareLaunchArgument('vesc_config', default_value=vesc_config),
        DeclareLaunchArgument('sensors_config', default_value=sensors_config),
        DeclareLaunchArgument('mux_config', default_value=mux_config),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # Joint state publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
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
            parameters=[LaunchConfiguration('vesc_config')]
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
            parameters=[LaunchConfiguration('sensors_config')]
        ),

        # Ackermann mux node
        Node(
            package='ackermann_mux',
            executable='ackermann_mux',
            name='ackermann_mux',
            parameters=[LaunchConfiguration('mux_config')],
            remappings=[('ackermann_cmd_out', 'ackermann_drive')]
        ),

        # RealSense Camera Node (Using default rs_launch.py)
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
                'pointcloud': True,
            }],
            remappings=[
                ('/camera/color/image_raw', '/camera/color/image_raw'),
                ('/camera/depth/image_raw', '/camera/depth/image_raw'),
                ('/camera/imu', '/camera/imu'),
                ('/camera/pointcloud', '/camera/depth/color/points')
            ]
        ),
    ])
