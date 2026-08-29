from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix


# ── 겹치는 노드 감시 ──────────────────────────────────────────────────────
# virtual.launch.py 의 시뮬레이터는 `name='can_bridge'` 로 실기 브리지의 **노드 이름을
# 그대로 뺏어 쓴다.** 둘 다 떠 있으면 같은 토픽에 번갈아 퍼블리시하고, 구독자는 실기값과
# 시뮬값을 섞어 받는다. ROS 는 이걸 오류로 보지 않는다. 20260829 에 시뮬레이터를 안 내린
# 채로 20 분간 실험 4 회를 날렸다 — 엔코더가 0°↔105° 로 튀고 대기압인 board 5 가
# 305 kPa 로 읽혔다. 띄우기 전에 막는다.
def _abort_if_conflicting(other_pattern, other_name, this_name):
    import os, subprocess
    r = subprocess.run(['pgrep', '-af', other_pattern], capture_output=True, text=True)
    if r.returncode != 0:
        return
    # pgrep -f 는 **명령줄 어디든** 걸린다 — 이름만 언급한 셸 한 줄(`pkill -f
    # virtual_powerpack` 같은 것)까지 잡아 엉뚱하게 막는다. argv[0] 의 basename 이
    # 정확히 일치하는, 진짜 그 실행 파일인 프로세스만 남긴다.
    hits = []
    for ln in r.stdout.splitlines():
        pid, _, cmd = ln.partition(' ')
        if not cmd or pid == str(os.getpid()):
            continue
        if os.path.basename(cmd.split()[0]) == other_pattern:
            hits.append(ln)
    if not hits:
        return
    procs = '\n'.join('      ' + ln for ln in hits)
    raise RuntimeError(
        f"\n\n  {other_name} 이(가) 이미 돌고 있다 — {this_name} 과 노드 이름이 겹친다.\n"
        f"{procs}\n\n"
        f"  둘 다 /pack2/can_bridge 라 같은 토픽에 번갈아 퍼블리시한다. 그 데이터는\n"
        f"  실기값과 시뮬값이 섞인 쓰레기다 (ROS 는 경고조차 안 한다).\n\n"
        f"      pkill -f {other_pattern}\n\n"
        f"  로 내리고 다시 띄울 것.\n")

def _setup(context, *_a, **_k):
    _abort_if_conflicting('virtual_powerpack', '시뮬레이터(virtual.launch.py)', '실기 브리지')
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

    # 축 선택: axis:=N 이면 **물리 채널 N 하나만** 돌린다.
    #
    # 채널별 부피·밸브는 채널을 하나씩 돌려야 잴 수 있다. 여럿을 같이 돌리면
    # 같은 레일을 나눠 쓰느라 차압이 흔들려 추정이 흩어진다 (6축 로그에서 같은
    # 채널이 창마다 0.15~2.45 배로 튀었다 — HANDOFF S-20).
    #
    # num_actuators=1 로 두고 axis0 의 gid 를 N 으로 돌려놓는 것과 같다.
    # 명시적으로 준 num_actuators / overrides 가 있으면 그쪽을 존중한다.
    axis = LaunchConfiguration('axis').perform(context)
    if axis:
        a = int(axis)
        npos = int(overrides.get('num_positive_channels', 6))
        overrides.setdefault('num_actuators', 1)
        overrides.setdefault('PositionController.axis0.pos_gid', a)
        overrides.setdefault('PositionController.axis0.neg_gid', npos + a)
        overrides.setdefault('PositionController.axis0.actuator_idx', a)

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
        output='screen',   # 'log' 였다 — 엔코더 캘리브레이션 실패 ERROR 가 터미널에
                           # 안 떠서 0도가 158도로 읽히는 것을 며칠 못 봤다,
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
        DeclareLaunchArgument('axis', default_value='',
            description='물리 채널 하나만 돌린다 (예: axis:=2 → 양압 ch2 / 음압 ch8 / '
                        '엔코더 board19). 채널별 부피·밸브를 하나씩 잴 때 쓴다.'),
        DeclareLaunchArgument('overrides', default_value='',
                              description="쉼표 구분 파라미터 오버라이드 (a.b=1.5,...)"),
        OpaqueFunction(function=_setup),
    ])
