import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # Retrieve the 'world' launch configuration value
    world_name = LaunchConfiguration('world').perform(context)
    
    pkg_path = get_package_share_directory('amr_robot')

    models_path = os.path.join(pkg_path, 'models')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + models_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = models_path

    xacro_file = os.path.join(
        pkg_path,
        'urdf',
        'amr_robot.urdf.xacro'
    )

    robot_description = xacro.process_file(
        xacro_file
    ).toxml()

    # Defaults (factory world)
    world_file = os.path.join(
        pkg_path,
        'worlds',
        'factory.world'
    )
    x_spawn = "0.0"
    y_spawn = "-2.5"
    z_spawn = "0.15"
    yaw_spawn = "1.5708"

    if world_name == 'opil_factory':
        try:
            opil_pkg = get_package_share_directory('opil_factory_world')
            world_file = os.path.join(opil_pkg, 'worlds', 'opil_factory.world')
        except Exception:
            world_file = os.path.join(pkg_path, 'worlds', 'opil_factory.world')
        x_spawn = "4.0"
        y_spawn = "4.0"
        z_spawn = "0.15"
        yaw_spawn = "0.0"
    elif world_name == 'warehouse':
        try:
            wh_pkg = get_package_share_directory('warehouse_world')
            world_file = os.path.join(wh_pkg, 'worlds', 'warehouse.world')
        except Exception:
            world_file = os.path.join(pkg_path, 'worlds', 'warehouse.world')
        x_spawn = "2.0"
        y_spawn = "2.0"
        z_spawn = "0.15"
        yaw_spawn = "0.0"

    # Override spawn coordinates if specified via launch configurations
    x_val = LaunchConfiguration('x').perform(context)
    y_val = LaunchConfiguration('y').perform(context)
    z_val = LaunchConfiguration('z').perform(context)
    yaw_val = LaunchConfiguration('yaw').perform(context)
    
    if x_val != "":
        x_spawn = x_val
    if y_val != "":
        y_spawn = y_val
    if z_val != "":
        z_spawn = z_val
    if yaw_val != "":
        yaw_spawn = yaw_val

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
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
            '--z', '0.1075',
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
                    x_spawn,
                    '-y',
                    y_spawn,
                    '-z',
                    z_spawn,
                    '-Y',
                    yaw_spawn
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
                    '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                    '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
                ],
                parameters=[{
                    'use_sim_time': True
                }],
                output='screen'
            )
        ]
    )

    return [
        robot_state_publisher,
        static_tf,
        gazebo,
        spawn_robot,
        bridge
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='factory',
            description='World to load: factory, opil_factory, or warehouse'
        ),
        DeclareLaunchArgument(
            'x',
            default_value='',
            description='Initial x spawn coordinate (leave empty for world default)'
        ),
        DeclareLaunchArgument(
            'y',
            default_value='',
            description='Initial y spawn coordinate (leave empty for world default)'
        ),
        DeclareLaunchArgument(
            'z',
            default_value='',
            description='Initial z spawn coordinate (leave empty for world default)'
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='',
            description='Initial yaw spawn orientation (leave empty for world default)'
        ),
        OpaqueFunction(function=launch_setup)
    ])