from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    pkg_share   = get_package_share_directory('can_powerpack')
    pkg_prefix  = get_package_prefix('can_powerpack')
    config_path  = os.path.join(pkg_share,  'config', 'powerpack_config.yaml')
    # 머신 생성 파라미터 — 있으면 손으로 쓴 설정 **뒤에** 병합해 덮어쓴다
    _share = os.path.dirname(os.path.dirname(config_path))
    fitted = [os.path.join(_share, 'config', n)
              for n in ('valve_params.yaml', 'pump_params.yaml')]
    fitted = [f for f in fitted if os.path.exists(f)]

    monitor_path = os.path.join(pkg_prefix, 'lib', 'can_powerpack', 'pp_monitor.py')
    logger_path  = os.path.join(pkg_prefix, 'lib', 'can_powerpack', 'pp_logger.py')
    setup_bash   = os.path.normpath(os.path.join(pkg_prefix, '..', 'setup.bash'))

    can_bridge = Node(
        package='can_powerpack',
        executable='can_bridge_node',
        name='can_bridge',
        namespace='pack2',
        output='log',
        parameters=[config_path, *fitted],
    )

    controller = Node(
        package='can_powerpack',
        executable='pp_controller',
        name='pp_controller',
        namespace='pack2',
        output='log',
        parameters=[config_path, *fitted],
    )

    logger = ExecuteProcess(
        cmd=['python3', logger_path],
        output='log',
    )

    monitor = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--maximize', '--',
            'bash', '-c',
            f'source {setup_bash} && python3 {monitor_path}; '
            f'echo "monitor exited — press Enter to close"; read',
        ],
        output='screen',
    )

    return LaunchDescription([
        can_bridge,
        controller,
        logger,
        monitor,
    ])
