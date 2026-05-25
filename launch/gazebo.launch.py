import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('amr_robot')
    xacro_file = os.path.join(pkg_path, 'urdf', 'amr_robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    world_file = os.path.join(pkg_path, 'worlds', 'factory.world')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # Publish base_footprint → base_link static transform
    # Nav2 expects base_footprint as the navigation frame
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0',
                   'base_link', 'base_footprint'],
        parameters=[{'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', 'robot_description',
                    '-name', 'amr_robot',
                    '-x', '0.0',
                    '-y', '-2.5',
                    '-z', '0.15',
                    '-Y', '1.5708'
                ],
                output='screen'
            )
        ]
    )

    bridge = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                    '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        static_tf,
        gazebo,
        spawn_robot,
        bridge,
    ])
