import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

def launch_setup(context, *args, **kwargs):
    # 1. Paths and Directories
    pkg_share = get_package_share_directory('amr_robot')

    # Resolve map parameter
    map_input = LaunchConfiguration('map').perform(context)
    if not os.path.isabs(map_input):
        map_name = map_input
        if not map_name.endswith('.yaml'):
            map_name += '.yaml'
        
        potential_path = os.path.join(pkg_share, 'maps', map_name)
        if os.path.exists(potential_path):
            map_path = potential_path
        else:
            map_path = os.path.abspath(map_input)
    else:
        map_path = map_input

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    # Lifecycle Nodes configuration
    # The nodes to be managed by the lifecycle manager
    lifecycle_nodes = [
        'map_server',
        'amcl',
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator'
    ]

    # Standard tf/tf_static topic remappings
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Map Server Node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': map_path}],
        remappings=remappings
    )

    # AMCL Localization Node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file],
        remappings=remappings
    )

    # Planner Server Node (Global path planning)
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
        remappings=remappings
    )

    # Controller Server Node (Local path execution & obstacle avoidance)
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=remappings + [('cmd_vel', '/cmd_vel')]
    )

    # Behavior Server Node (Recovery behaviors)
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
        remappings=remappings + [('cmd_vel', '/cmd_vel')]
    )

    # BT Navigator Node (Behavior Tree execution)
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
        remappings=remappings
    )

    # Lifecycle Manager Node (Orchestrates startup of all lifecycle nodes)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes,
            'bond_timeout': 0.0
        }]
    )

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return [
        SetParameter('use_sim_time', use_sim_time),
        map_server_node,
        amcl_node,
        planner_server_node,
        controller_server_node,
        behavior_server_node,
        bt_navigator_node,
        lifecycle_manager_node,
        rviz_node
    ]

def generate_launch_description():
    pkg_share = get_package_share_directory('amr_robot')

    default_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'nav2.rviz')

    # Force stdout buffering to ensure logs appear sequentially
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # Declare Launch Arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='demo_factory',
        description='Full path to occupancy grid map yaml file OR map name (e.g. demo_factory, opil_factory)'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config_path,
        description='Full path to the RViz config file to use'
    )

    # Create Launch Description and return actions
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Add the actions to declare launch arguments FIRST
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Add the OpaqueFunction setup
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld

