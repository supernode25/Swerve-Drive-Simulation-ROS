"""sim_nav.launch.py — fake robot + Nav2 (midpro FSM controller, Smac 2D global, Smac Hybrid-A* detour)

    ros2 launch midpro_bringup sim_nav.launch.py                       # map + AMCL, detour = follow global plan, wall clock
    ros2 launch midpro_bringup sim_nav.launch.py sim_time:=true        # simulator drives /clock (needed only for RTF != 1)
    ros2 launch midpro_bringup sim_nav.launch.py localization:=gt      # sim ground-truth TF (no AMCL)
    ros2 launch midpro_bringup sim_nav.launch.py detour:=true          # Hybrid-A* detour planner instead of the global plan
    ros2 launch midpro_bringup sim_nav.launch.py slam:=true            # slam_toolbox mapping
    ros2 launch midpro_bringup sim_nav.launch.py world_map:=<test_hall.yaml> map:=<test_hall.yaml> init_x:=1.5 init_y:=1.5
    ros2 launch midpro_bringup sim_nav.launch.py rviz:=false dash:=false
    ros2 launch midpro_bringup sim_nav.launch.py obstacles:=/path/to/obstacles.yaml
    # real site map: same yaml for the sim world and for Nav2 (see README for the full command)
    ros2 launch midpro_bringup sim_nav.launch.py world_map:=<maps>/ecoproBM_matterport.yaml \
        map:=<maps>/ecoproBM_matterport.yaml init_x:=34.3 init_y:=0.95 obstacles:=""

The simulator always ray-casts against `world_map` (ground truth). In SLAM mode the map
Nav2 uses is the one slam_toolbox builds, so they differ until the area is explored.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = get_package_share_directory('midpro_bringup')
    sim = get_package_share_directory('midpro_sim')
    desc = get_package_share_directory('midpro_description')

    params_file = LaunchConfiguration('params_file')
    world_map = LaunchConfiguration('world_map')
    nav_map = LaunchConfiguration('map')
    slam = LaunchConfiguration('slam')
    rviz = LaunchConfiguration('rviz')
    dash = LaunchConfiguration('dash')
    rviz_dark = LaunchConfiguration('rviz_dark')
    rviz_cfg = PythonExpression(["'", os.path.join(bringup, 'rviz', 'midpro_dark.rviz'), "' if '", rviz_dark, "' == 'true' else '", os.path.join(bringup, 'rviz', 'midpro.rviz'), "'"])
    obstacles = LaunchConfiguration('obstacles')
    autostart = LaunchConfiguration('autostart')
    detour = LaunchConfiguration('detour')
    hybrid_fallback = LaunchConfiguration('hybrid_fallback')
    run_detour_server = PythonExpression(["'", detour, "' == 'true' or ('", hybrid_fallback, "' == 'true' and not '", LaunchConfiguration('profile'), "'.startswith('v028'))"])
    localization = LaunchConfiguration('localization')
    use_gt = PythonExpression(["'", localization, "' == 'gt'"])
    init_x = LaunchConfiguration('init_x')
    init_y = LaunchConfiguration('init_y')
    init_yaw = LaunchConfiguration('init_yaw')
    sim_time = LaunchConfiguration('sim_time')
    use_sim_time = {'use_sim_time': ParameterValue(sim_time, value_type=bool)}

    declare = [
        DeclareLaunchArgument('profile', default_value='v028_narrow',
                              description='v028 | v028_tight (closer clearances) | v028_narrow (robot width + 10 cm gaps) | current (latest)'),
        DeclareLaunchArgument('params_file', default_value=PythonExpression([
            "{'v028': '", os.path.join(bringup, 'params', 'nav2_params_v028.yaml'),
            "', 'v028_tight': '", os.path.join(bringup, 'params', 'nav2_params_v028_tight.yaml'),
            "', 'v028_narrow': '", os.path.join(bringup, 'params', 'nav2_params_v028_narrow.yaml'),
            "'}.get('", LaunchConfiguration('profile'), "', '", os.path.join(bringup, 'params', 'nav2_params.yaml'), "')"])),
        DeclareLaunchArgument('world_map', default_value=os.path.join(bringup, 'maps', 'ecoproBM_matterport_clean.yaml'),
                              description='ground-truth map the simulator ray-casts against'),
        DeclareLaunchArgument('map', default_value=os.path.join(bringup, 'maps', 'ecoproBM_matterport_clean.yaml'),
                              description='map served to Nav2 (localization mode)'),
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('dash', default_value='true', description='PyQt5 speed dashboard (midpro_dash)'),
        DeclareLaunchArgument('goal_viz', default_value='true', description='project the robot footprint on the goal in RViz'),
        DeclareLaunchArgument('safety', default_value='guard', description='guard: direction-aware midpro_safety/obstacle_guard | monitor: nav2 collision_monitor'),
        DeclareLaunchArgument('rviz_dark', default_value='true', description='dark RViz view config (midpro_dark.rviz)'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('obstacles', default_value=''),
        DeclareLaunchArgument('hybrid_fallback', default_value='true',
                              description='run the Hybrid-A* /detour server as a fallback for footprint-infeasible global plans'),
        DeclareLaunchArgument('detour', default_value='false',
                              description='true: Hybrid-A* detour planner server; false: follow the Nav2 global plan in detour state'),
        DeclareLaunchArgument('localization', default_value='amcl', description='amcl | gt (sim ground truth TF)'),
        DeclareLaunchArgument('model', default_value='midpro', description='midpro | swerve (robot visual)'),
        DeclareLaunchArgument('sim_time', default_value='false',
                              description='false: all nodes on wall clock (recommended); true: simulator drives /clock'),
        DeclareLaunchArgument('init_x', default_value='34.3'),
        DeclareLaunchArgument('init_y', default_value='0.95'),
        DeclareLaunchArgument('init_yaw', default_value='0.0'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
    ]

    # ---------------------------------------------------------------- simulator + description
    sim_nodes = [
        # The simulator PRODUCES /clock, so it must run on wall clock (use_sim_time: False).
        Node(package='midpro_sim', executable='fake_robot', name='fake_robot', output='screen',
             parameters=[os.path.join(sim, 'config', 'sim_params.yaml'),
                         {'world_map_yaml': world_map, 'obstacle_yaml': obstacles, 'use_sim_time': False,
                          'publish_clock': ParameterValue(sim_time, value_type=bool),
                          'publish_map_odom_tf': ParameterValue(use_gt, value_type=bool),
                          'initial_x': ParameterValue(init_x, value_type=float),
                          'initial_y': ParameterValue(init_y, value_type=float),
                          'initial_yaw': ParameterValue(init_yaw, value_type=float)}]),
        Node(package='midpro_sim', executable='detour_bridge', name='detour_bridge', output='screen',
             parameters=[os.path.join(sim, 'config', 'sim_params.yaml'), use_sim_time],
             condition=IfCondition(run_detour_server)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(desc, 'launch', 'description.launch.py')),
            launch_arguments={'use_sim_time': sim_time, 'model': LaunchConfiguration('model')}.items()),
    ]

    # ---------------------------------------------------------------- localization: map + AMCL
    localization = GroupAction(condition=UnlessCondition(slam), actions=[
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[params_file, {'yaml_filename': nav_map, **use_sim_time}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             condition=UnlessCondition(use_gt),
             parameters=[params_file, use_sim_time,
                         {'initial_pose.x': ParameterValue(init_x, value_type=float),
                          'initial_pose.y': ParameterValue(init_y, value_type=float),
                          'initial_pose.yaw': ParameterValue(init_yaw, value_type=float)}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
             output='screen', condition=UnlessCondition(use_gt),
             parameters=[{'autostart': autostart, 'node_names': ['map_server', 'amcl'], **use_sim_time}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
             output='screen', condition=IfCondition(use_gt),
             parameters=[{'autostart': autostart, 'node_names': ['map_server'], **use_sim_time}]),
    ])

    # ---------------------------------------------------------------- SLAM: slam_toolbox (publishes /map and map->odom)
    slam_nodes = GroupAction(condition=IfCondition(slam), actions=[
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
             parameters=[os.path.join(bringup, 'params', 'slam_toolbox.yaml'), use_sim_time]),
    ])

    # ---------------------------------------------------------------- Nav2 core
    bt_dir = os.path.join(bringup, 'behavior_trees')
    bt_xml = PythonExpression(["'", bt_dir, "/midpro_nav_to_pose_detour.xml' if '", detour, "' == 'true' else ('",
                               bt_dir, "/midpro_nav_to_pose_v028.xml' if '", LaunchConfiguration('profile'), "'.startswith('v028') else '",
                               bt_dir, "/midpro_nav_to_pose.xml')"])
    nav_nodes = [
        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen',
             parameters=[params_file, use_sim_time,
                         {'FollowPath.detour_source': PythonExpression(["'planner' if '", detour, "' == 'true' else 'global_plan'"])}],
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=[params_file, use_sim_time]),
        # planner #2: detour (ex eband). Own namespace because planner_server hardcodes "global_costmap".
        GroupAction(condition=IfCondition(run_detour_server), actions=[
            PushRosNamespace('detour'),
            Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
                 parameters=[params_file, use_sim_time],
                 remappings=[('/detour/map', '/map'), ('/detour/scan', '/scan'),
                             ('/detour/tf', '/tf'), ('/detour/tf_static', '/tf_static')]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_detour',
                 output='screen', parameters=[{'autostart': autostart, 'node_names': ['planner_server'], 'bond_timeout': 0.0, **use_sim_time}]),
        ]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen',
             parameters=[params_file, use_sim_time]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
             parameters=[params_file, {'default_nav_to_pose_bt_xml': bt_xml, **use_sim_time}]),
        # cmd_vel chain: controller -> /cmd_vel_nav -> [obstacle_guard | collision_monitor] -> /cmd_vel_cm -> velocity_smoother -> /cmd_vel
        Node(package='midpro_safety', executable='obstacle_guard', name='obstacle_guard', output='screen',
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('safety'), "' == 'guard'"])),
             parameters=[os.path.join(get_package_share_directory('midpro_safety'), 'config', 'obstacle_guard.yaml'), use_sim_time],
             remappings=[('cmd_in', 'cmd_vel_nav'), ('cmd_out', 'cmd_vel_cm'), ('scan', 'scan'), ('odom', 'odom')]),
        GroupAction(condition=IfCondition(PythonExpression(["'", LaunchConfiguration('safety'), "' == 'monitor'"])), actions=[
            Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor', output='screen',
                 parameters=[params_file, use_sim_time]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_safety',
                 output='screen', parameters=[{'autostart': autostart, 'node_names': ['collision_monitor'], **use_sim_time}]),
        ]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother', output='screen',
             parameters=[params_file, use_sim_time],
             remappings=[('cmd_vel', 'cmd_vel_cm'), ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             output='screen',
             parameters=[{'autostart': autostart, **use_sim_time,
                          'node_names': ['controller_server', 'planner_server', 'behavior_server',
                                         'bt_navigator', 'velocity_smoother']}]),
    ]

    rviz_node = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                     arguments=['-d', rviz_cfg, '-style', 'Fusion'],
                     additional_env={'QT_QPA_PLATFORMTHEME': 'qt5ct'},   # panels follow the qt5ct palette if installed
                     parameters=[use_sim_time], condition=IfCondition(rviz))
    # live speed/accel/steering dashboard (requires python3-pyqt5 python3-pyqtgraph)
    dash_node = Node(package='midpro_dash', executable='speed_dash', name='speed_dash', output='screen',
                     condition=IfCondition(dash))

    goal_viz_node = Node(package='midpro_dash', executable='goal_footprint', name='goal_footprint', output='screen',
                         condition=IfCondition(LaunchConfiguration('goal_viz')))

    return LaunchDescription(declare + sim_nodes + [localization, slam_nodes] + nav_nodes + [rviz_node, dash_node, goal_viz_node])
