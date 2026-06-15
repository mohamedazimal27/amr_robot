import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('amr_robot')
    
    # Launch Configurations
    world_config = LaunchConfiguration('world')
    map_config = LaunchConfiguration('map')
    
    # Declare Launch Arguments
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='opil_factory',
        description='World to load: factory, opil_factory, or warehouse'
    )
    
    declare_map = DeclareLaunchArgument(
        'map',
        default_value='opil_factory.yaml',
        description='Map file to load for navigation'
    )

    # 1. Gazebo Simulation launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_config
        }.items()
    )

    # 2. Navigation & Localization stack launch (AMCL, Nav2, Map Server)
    navigation_launch = TimerAction(
        period=5.0, # Give Gazebo a moment to start and spawn robot
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_path, 'launch', 'navigation.launch.py')
                ),
                launch_arguments={
                    'map': map_config,
                    'use_sim_time': 'true',
                    'autostart': 'true'
                }.items()
            )
        ]
    )

    # 3. Perception Node: QR Detector
    qr_detector_node = Node(
        package='amr_robot',
        executable='qr_detector',
        name='qr_detector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'processing_rate': 10.0,
            'velocity_threshold': 0.05,
            'center_gate_tolerance': 0.20
        }]
    )

    # 4. Inventory Logger Node
    inventory_logger_node = Node(
        package='amr_robot',
        executable='inventory_logger',
        name='inventory_logger',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'db_path': '/home/mohamed-azimal/ros2_ws/src/amr_robot/maps/inventory.db'
        }]
    )

    # 5. Metrics Evaluator Node
    metrics_evaluator_node = Node(
        package='amr_robot',
        executable='metrics_evaluator',
        name='metrics_evaluator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'db_path': '/home/mohamed-azimal/ros2_ws/src/amr_robot/maps/inventory.db',
            'output_dir': '/home/mohamed-azimal/ros2_ws/src/amr_robot/docs'
        }]
    )

    # 6. Mission Manager Node (with 15s delay to allow Nav2, AMCL, and maps to fully load & stabilize)
    mission_manager_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='amr_robot',
                executable='mission_manager',
                name='mission_manager',
                output='screen',
                parameters=[{
                    'use_sim_time': True
                }]
            )
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_world)
    ld.add_action(declare_map)
    ld.add_action(gazebo_launch)
    ld.add_action(navigation_launch)
    ld.add_action(qr_detector_node)
    ld.add_action(inventory_logger_node)
    ld.add_action(metrics_evaluator_node)
    ld.add_action(mission_manager_node)
    
    return ld
