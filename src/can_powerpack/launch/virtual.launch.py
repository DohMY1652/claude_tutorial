"""CAN 하드웨어 없이 pp_controller 를 돌리는 가상 시스템 런치.

  pp_controller  ←→  virtual_powerpack (노드명 can_bridge)

virtual_powerpack 은 CanBridge 와 동일한 토픽(board/sensors, board/currents,
board/analog, board/pwm_cmd)을 쓰므로 컨트롤러 쪽 수정이 전혀 필요 없다.

파라미터 주입 전략:
  가상 시스템은 pp_controller 와 **같은** 채널 구성 / 센서 캘리브레이션 /
  밸브 13-파라미터를 써야 센서 왕복과 밸브 모델이 일치한다. 값을 두 곳에
  복제하지 않기 위해, 여기서 powerpack_config.yaml 을 직접 읽어
  pp_controller 섹션 + can_bridge 섹션(엔코더 캘리브레이션)을 평탄화해서
  virtual_powerpack 에 넘긴다. virtual_powerpack.yaml 의 물리 파라미터가
  마지막에 덮어쓴다.

사용:
  ros2 launch can_powerpack virtual.launch.py
  ros2 launch can_powerpack virtual.launch.py actuator_connected:=false
  ros2 launch can_powerpack virtual.launch.py monitor:=true
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAMESPACE = 'pack2'


def _flatten(prefix, value, out):
    """nested dict → ROS2 파라미터 이름(a.b.c) 으로 평탄화. 리스트는 그대로 둔다."""
    if isinstance(value, dict):
        for k, v in value.items():
            _flatten(f'{prefix}.{k}' if prefix else str(k), v, out)
    else:
        out[prefix] = value


def _load_params(path, node_key):
    """yaml 파일의 /<ns>/<node>: ros__parameters 섹션을 평탄화한 dict 로 반환."""
    with open(path) as f:
        doc = yaml.safe_load(f) or {}
    section = doc.get(node_key, {}).get('ros__parameters', {})
    flat = {}
    _flatten('', section, flat)
    return flat


def _launch_setup(context, *_args, **_kwargs):
    pkg_share = get_package_share_directory('can_powerpack')
    ctrl_cfg = os.path.join(pkg_share, 'config', 'powerpack_config.yaml')
    virt_cfg = os.path.join(pkg_share, 'config', 'virtual_powerpack.yaml')
    # 머신 생성 파라미터 파일들 — 있으면 손으로 쓴 설정 **뒤에** 병합해 덮어쓴다
    fitted = [os.path.join(pkg_share, 'config', n)
              for n in ('valve_params.yaml', 'pump_params.yaml', 'encoder_params.yaml')]
    fitted = [f for f in fitted if os.path.exists(f)]

    actuator_connected = LaunchConfiguration('actuator_connected').perform(context) == 'true'
    show_monitor = LaunchConfiguration('monitor').perform(context) == 'true'
    control_mode_str = LaunchConfiguration('control_mode').perform(context)
    solver_str = LaunchConfiguration('solver').perform(context)
    overrides_str = LaunchConfiguration('overrides').perform(context)

    # 가상 시스템 파라미터: controller 설정 → can_bridge 설정 → sim 물리 설정 순으로 덮어쓰기
    sim_params = {}
    sim_params.update(_load_params(ctrl_cfg, f'/{NAMESPACE}/pp_controller'))
    sim_params.update(_load_params(ctrl_cfg, f'/{NAMESPACE}/can_bridge'))
    sim_params.update(_load_params(virt_cfg, f'/{NAMESPACE}/can_bridge'))
    for f in fitted:
        sim_params.update(_load_params(f, f'/{NAMESPACE}/can_bridge'))

    # 가상 플랜트도 컨트롤러와 **같은 밸브 파라미터**를 써야 한다.
    # valve_params.yaml 은 /pack2/pp_controller 아래에 있어서 can_bridge 섹션만
    # 읽던 위 루프에는 잡히지 않았고, 그 결과 시뮬 플랜트는 지금까지 피팅값을
    # 한 번도 쓰지 않고 코드 기본값(A_max 0.2845, alpha_shape 3884.2) 으로 돌았다.
    # 컨트롤러가 보는 밸브와 시뮬 안의 밸브가 서로 다른 물건이었다는 뜻이다.
    for f in fitted:
        if f.endswith('valve_params.yaml'):
            sim_params.update(_load_params(f, f'/{NAMESPACE}/pp_controller'))


    sim_params['actuator_connected'] = actuator_connected

    ctrl_overrides = {'actuator_connected': actuator_connected}
    if control_mode_str:
        ctrl_overrides['control_mode'] = int(control_mode_str)
    if solver_str:
        # 같은 빌드로 qp/mppi 를 A/B 하기 위한 오버라이드. 하네스가 비결정론적이라
        # 빌드를 바꿔 비교하면 원인 분리가 안 된다.
        ctrl_overrides['MPC_parameters.solver'] = solver_str
    for item in filter(None, (x.strip() for x in overrides_str.split(','))):
        # 'a.b=1.5' 형태. 튜닝 스윕을 같은 빌드로 돌리기 위한 통로다.
        k, _, v = item.partition('=')
        try:
            ctrl_overrides[k.strip()] = int(v) if v.strip().lstrip('-').isdigit() else float(v)
        except ValueError:
            ctrl_overrides[k.strip()] = v.strip() in ('true', 'True') if v.strip() in (
                'true', 'True', 'false', 'False') else v.strip()

    # 축 선택: axis:=N 이면 **물리 채널 N 하나만** 돌린다.
    #
    # 채널별 부피·밸브는 채널을 하나씩 돌려야 잴 수 있다. 여럿을 같이 돌리면
    # 같은 레일을 나눠 쓰느라 차압이 흔들려 추정이 흩어진다 (6축 로그에서 같은
    # 채널이 창마다 0.15~2.45 배로 튀었다 — HANDOFF S-20).
    #
    # num_actuators=1 로 두고 axis0 의 gid 를 N 으로 돌려놓는 것과 같다.
    # 명시적으로 준 num_actuators / ctrl_overrides 가 있으면 그쪽을 존중한다.
    axis = LaunchConfiguration('axis').perform(context)
    if axis:
        a = int(axis)
        npos = int(ctrl_overrides.get('num_positive_channels', 6))
        ctrl_overrides.setdefault('num_actuators', 1)
        ctrl_overrides.setdefault('PositionController.axis0.pos_gid', a)
        ctrl_overrides.setdefault('PositionController.axis0.neg_gid', npos + a)
        ctrl_overrides.setdefault('PositionController.axis0.actuator_idx', a)

    virtual_system = Node(
        package='can_powerpack',
        executable='virtual_powerpack',
        name='can_bridge',          # CanBridge 자리를 그대로 대체
        namespace=NAMESPACE,
        output='screen',
        parameters=[sim_params],
    )

    controller = Node(
        package='can_powerpack',
        executable='pp_controller',
        name='pp_controller',
        namespace=NAMESPACE,
        output='screen',
        parameters=[ctrl_cfg, *fitted, ctrl_overrides],
    )

    # 시뮬도 실기와 **같은 형식의 CSV** 를 남긴다 — 그래야 둘을 나란히 비교할 수 있다.
    pkg_prefix = get_package_prefix('can_powerpack')
    logger_path = os.path.join(pkg_prefix, 'lib', 'can_powerpack', 'pp_logger.py')
    actions = [virtual_system, controller,
               ExecuteProcess(cmd=['python3', logger_path], output='log')]

    if show_monitor:
        monitor_path = os.path.join(pkg_prefix, 'lib', 'can_powerpack', 'pp_monitor.py')
        setup_bash = os.path.normpath(os.path.join(pkg_prefix, '..', 'setup.bash'))
        actions.append(ExecuteProcess(
            cmd=['gnome-terminal', '--maximize', '--', 'bash', '-c',
                 f'source {setup_bash} && python3 {monitor_path}; '
                 f'echo "monitor exited — press Enter to close"; read'],
            output='screen',
        ))

    return actions



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

def generate_launch_description():
    _abort_if_conflicting('can_bridge_node', '실기 브리지(control.launch.py)', '시뮬레이터')
    return LaunchDescription([
        DeclareLaunchArgument(
            'actuator_connected', default_value='true',
            description='true: 액추에이터 각도 동역학 활성(위치 제어 폐루프). '
                        'false: 각도 고정, 고정 탱크 부피로 압력 추종만 테스트.'),
        DeclareLaunchArgument(
            'control_mode', default_value='',
            description='비우면 yaml 값 사용. 0=압력, 1=위치(휴리스틱), 2=위치(최적화 생성기).'),
        DeclareLaunchArgument(
            'solver', default_value='',
            description="비우면 yaml 값 사용. 'qp' 또는 'mppi'. 채널 MPC 의 솔버를 고른다."),
        DeclareLaunchArgument(
            'axis', default_value='',
            description='물리 채널 하나만 돌린다 (예: axis:=2 → 양압 ch2 / 음압 ch8 / 엔코더 board19). 채널별 부피·밸브를 하나씩 잴 때 쓴다.'),
        DeclareLaunchArgument(
            'overrides', default_value='',
            description="쉼표로 구분한 파라미터 오버라이드. 예: "
                        "'MPC_parameters.mppi_w_effort=0.5,MPC_parameters.mppi_lambda=0.5'"),
        DeclareLaunchArgument(
            'monitor', default_value='false',
            description='true: pp_monitor.py 를 별도 터미널로 띄운다 (gnome-terminal 필요).'),
        OpaqueFunction(function=_launch_setup),
    ])
