import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('amr_robot')

    xacro_file = os.path.join(
        pkg_path,
        'urdf',
        'amr_robot.urdf.xacro'
    )

    robot_description = xacro.process_file(
        xacro_file
    ).toxml()

    world_file = os.path.join(
        pkg_path,
        'worlds',
        'factory.world'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    joint_state_publisher = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    parameters=[{
        'use_sim_time': True
    }]
)


    # base_footprint -> base_link
    # Keep base_link as physical root
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0.0425',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'base_link'
        ],
        parameters=[{
            'use_sim_time': True
        }]
    )


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(
                    'ros_gz_sim'
                ),
                'launch',
                'gz_sim.launch.py'
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
                    '-topic',
                    'robot_description',

                    '-name',
                    'amr_robot',

                    '-x',
                    '0.0',

                    '-y',
                    '-2.5',

                    '-z',
                    '0.15',

                    '-Y',
                    '1.5708'
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

                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'

                ],
                parameters=[{
                    'use_sim_time': True
                }],
                output='screen'
            )

        ]
    )


    return LaunchDescription([

        robot_state_publisher,
        joint_state_publisher,
        static_tf,
        gazebo,
        spawn_robot,
        bridge

    ])