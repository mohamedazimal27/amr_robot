import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('amr_robot')

    # 1. Perception Node: QR Detector
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

    # 2. Inventory Logger Node
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

    # 3. Metrics Evaluator Node
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

    # 4. Mission Manager Node
    mission_manager_node = Node(
        package='amr_robot',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    ld = LaunchDescription()
    ld.add_action(qr_detector_node)
    ld.add_action(inventory_logger_node)
    ld.add_action(metrics_evaluator_node)
    ld.add_action(mission_manager_node)
    
    return ld
