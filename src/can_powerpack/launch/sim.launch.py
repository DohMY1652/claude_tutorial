"""Simulation launch: pp_controller + pneumatic_sim (no can_bridge_node)."""
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share  = get_package_share_directory('can_powerpack')
    config_dir = os.path.join(pkg_share, 'config')

    ctrl_config = os.path.join(config_dir, 'powerpack_config.yaml')
    sim_config  = os.path.join(config_dir, 'sim_config.yaml')

    controller_node = Node(
        package='can_powerpack',
        executable='pp_controller',
        name='pp_controller',
        namespace='pack2',
        output='screen',
        parameters=[ctrl_config],
    )

    sim_node = Node(
        package='can_powerpack',
        executable='pneumatic_sim',
        name='pneumatic_sim',
        namespace='pack2',
        output='screen',
        parameters=[sim_config],
    )

    return LaunchDescription([controller_node, sim_node])
