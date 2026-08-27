from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def _setup(context, *_a, **_k):
    pkg_share   = get_package_share_directory('can_powerpack')
    pkg_prefix  = get_package_prefix('can_powerpack')
    config_path  = os.path.join(pkg_share,  'config', 'powerpack_config.yaml')
    # 머신 생성 파라미터 — 있으면 손으로 쓴 설정 **뒤에** 병합해 덮어쓴다
    _share = os.path.dirname(os.path.dirname(config_path))
    fitted = [os.path.join(_share, 'config', n)
              for n in ('valve_params.yaml', 'pump_params.yaml', 'encoder_params.yaml')]
    fitted = [f for f in fitted if os.path.exists(f)]

    monitor_path = os.path.join(pkg_prefix, 'lib', 'can_powerpack', 'pp_monitor.py')
    logger_path  = os.path.join(pkg_prefix, 'lib', 'can_powerpack', 'pp_logger.py')
    setup_bash   = os.path.normpath(os.path.join(pkg_prefix, '..', 'setup.bash'))

    # 실기에서도 **같은 빌드로 솔버를 바꿔** 비교할 수 있어야 한다 (qp | mppi | mppi_system).
    overrides = {}
    solver = LaunchConfiguration('solver').perform(context)
    if solver:
        overrides['MPC_parameters.solver'] = solver
    nact = LaunchConfiguration('num_actuators').perform(context)
    if nact:
        overrides['num_actuators'] = int(nact)
    act = LaunchConfiguration('actuator_connected').perform(context)
    if act:
        overrides['actuator_connected'] = (act.lower() == 'true')
    for item in filter(None, (x.strip() for x in
                              LaunchConfiguration('overrides').perform(context).split(','))):
        k, _, v = item.partition('=')
        vs = v.strip()
        try:
            overrides[k.strip()] = int(vs) if vs.lstrip('-').isdigit() else float(vs)
        except ValueError:
            overrides[k.strip()] = (vs in ('true', 'True')) if vs in (
                'true', 'True', 'false', 'False') else vs

    # can_bridge 에도 **자기가 선언한 키만** 넘긴다. overrides 를 통째로 넘기면
    # MPC_parameters.* 처럼 브리지가 선언하지 않은 파라미터가 섞여 노드가 뜨지 않는다.
    #
    # num_actuators 가 특히 중요하다 — 브리지는 이 값으로 활성 엔코더 보드를
    # (17 .. 17+N-1) 로 정한다. 넘기지 않으면 `num_actuators:=1` 로 1축 시험을 해도
    # 브리지는 yaml 의 6 을 그대로 써서 board 17~22 를 기대하고, 20~22 에 대해
    # 캘리브레이션 경고와 수신 없음 오류를 쏟는다 (그 시험에는 쓰이지도 않는 보드다).
    BRIDGE_KEYS = ('num_actuators', 'can_channel', 'current_mode', 'control_type',
                   'pwm_watchdog_ms', 'can_rx_watchdog_ms',
                   'can_tx_fallback_ms', 'can_tx_min_interval_ms', 'can_diag_period_s')
    bridge_overrides = {k: v for k, v in overrides.items() if k in BRIDGE_KEYS}

    can_bridge = Node(
        package='can_powerpack',
        executable='can_bridge_node',
        name='can_bridge',
        namespace='pack2',
        output='log',
        parameters=[config_path, *fitted, bridge_overrides],
    )

    controller = Node(
        package='can_powerpack',
        executable='pp_controller',
        name='pp_controller',
        namespace='pack2',
        output='log',
        parameters=[config_path, *fitted, overrides],
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

    return [can_bridge, controller, logger, monitor]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('solver', default_value='',
                              description="비우면 yaml 값. qp | mppi | mppi_system"),
        DeclareLaunchArgument('num_actuators', default_value='',
                              description='축(=채널쌍) 수. 1..6. 비우면 yaml 값'),
        DeclareLaunchArgument('actuator_connected', default_value='',
                              description='true|false. 비우면 yaml 값'),
        DeclareLaunchArgument('overrides', default_value='',
                              description="쉼표 구분 파라미터 오버라이드 (a.b=1.5,...)"),
        OpaqueFunction(function=_setup),
    ])
