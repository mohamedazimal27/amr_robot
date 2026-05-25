import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('amr_robot')
    slam_params = os.path.join(pkg_path, 'config', 'slam_toolbox_params.yaml')

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_params,
            {'use_sim_time': True}
        ],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        slam_toolbox,
        rviz,
    ])
