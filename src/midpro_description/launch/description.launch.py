"""description.launch.py — robot_state_publisher (+ joint_state_publisher for steerable wheels)

    model:=midpro            box base with the midpro 1.4 footprint (default)
    model:=swerve            parameterised 4-wheel swerve platform (MobED-spec dimensions by default)
    mesh_file:=/path/x.dae   optional body mesh for model:=swerve (your own / licensed CAD)

The URDF is generated with the xacro Python API (not a shell command), so workspace paths containing
spaces or brackets (e.g. "midpro_nav2_ws (6)" from a repeated browser download) work.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _nodes(context):
    pkg = get_package_share_directory('midpro_description')
    model = LaunchConfiguration('model').perform(context)
    mesh_file = LaunchConfiguration('mesh_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1', 'yes')
    fname = 'swerve_platform.urdf.xacro' if model == 'swerve' else 'midpro.urdf.xacro'
    robot_description = xacro.process_file(os.path.join(pkg, 'urdf', fname),
                                           mappings={'mesh_file': mesh_file}).toxml()
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]),
        # publishes zero angles for the continuous steer/wheel joints so RViz has a full TF tree
        Node(package='joint_state_publisher', executable='joint_state_publisher', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('model', default_value='midpro'),
        DeclareLaunchArgument('mesh_file', default_value=''),
        OpaqueFunction(function=_nodes),
    ])
