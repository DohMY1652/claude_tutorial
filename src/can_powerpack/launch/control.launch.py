from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('can_powerpack')
    config_path = os.path.join(pkg_share, 'config', 'powerpack_config.yaml')

    can_bridge_node = Node(
        package='can_powerpack',
        executable='can_bridge_node',
        name='can_bridge',
        namespace='pack2',
        output='screen',
        parameters=[config_path],
    )

    controller_node = Node(
        package='can_powerpack',
        executable='pp_controller',
        name='pp_controller',
        namespace='pack2',
        output='screen',
        parameters=[config_path],
    )

    return LaunchDescription([
        can_bridge_node,
        controller_node,
    ])
