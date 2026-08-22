#!/usr/bin/env bash
# ============================================================================
# preflight.sh — 실기 컴퓨터에서 처음 돌릴 때 확인할 것들
#
# 이 저장소의 성능 수치는 **개발 기계(32코어)** 에서 측정한 것이다. 실기 기계가 다르면
# 실시간 예산이 달라지므로 표본 수·스레드 설정을 바꿔야 한다. 이 스크립트가 그 판단
# 재료를 모아 준다. 아무것도 바꾸지 않고 읽기만 한다.
#
# 사용: bash src/can_powerpack/scripts/preflight.sh
# ============================================================================
set -u

ok()   { printf '  \033[32m✓\033[0m %s\n' "$1"; }
warn() { printf '  \033[33m!\033[0m %s\n' "$1"; }
bad()  { printf '  \033[31m✗\033[0m %s\n' "$1"; }
hdr()  { printf '\n\033[1m%s\033[0m\n' "$1"; }

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

hdr "1. 기계"
CORES=$(nproc)
echo "  호스트   : $(hostname)"
echo "  코어     : ${CORES}"
echo "  커널     : $(uname -r)"
if [ "$CORES" -ge 16 ]; then
  ok "코어 충분 — 개발 기계(32)와 비슷하다. 기본 설정을 그대로 써도 된다"
elif [ "$CORES" -ge 8 ]; then
  warn "코어 ${CORES}개 — 개발 기계(32)보다 적다. 아래 3절 권고를 볼 것"
else
  bad "코어 ${CORES}개 — 실시간 예산이 빠듯하다. 아래 3절을 반드시 적용할 것"
fi

hdr "2. 소프트웨어"
if [ -n "${ROS_DISTRO:-}" ]; then
  ok "ROS_DISTRO=${ROS_DISTRO} (source 됨)"
else
  bad "ROS 가 source 되지 않았다 — 'source /opt/ros/foxy/setup.bash' 먼저"
fi
python3 -c 'import numpy, yaml' 2>/dev/null \
  && ok "python3 numpy·yaml 있음 (피팅 코드에 필요)" \
  || bad "python3 numpy 또는 yaml 없음 — 피팅 코드가 안 돈다"
[ -d /usr/include/eigen3 ] && ok "eigen3 있음" || warn "eigen3 헤더를 못 찾았다 (libeigen3-dev)"

# Kvaser canlib — 없으면 can_bridge_node 가 **아예 빌드되지 않는다**
if ldconfig -p 2>/dev/null | grep -q libcanlib; then
  ok "Kvaser canlib 있음 → can_bridge_node 빌드된다"
else
  bad "Kvaser canlib 없음 → can_bridge_node 가 빌드되지 않는다 (실기 불가). 드라이버 설치 필요"
fi

hdr "3. 실시간 예산 권고 (코어 ${CORES}개 기준)"
PERIOD_MS=$(grep -oP '^\s*period_ms:\s*\K[0-9]+' \
  "${WS}/src/can_powerpack/config/powerpack_config.yaml" 2>/dev/null | head -1)
PERIOD_MS=${PERIOD_MS:-2}
echo "  현재 period_ms = ${PERIOD_MS} ($((1000 / PERIOD_MS)) Hz), 틱 예산 $((PERIOD_MS * 1000)) us"
echo
echo "  개발 기계 계측 (32코어, 12스레드):"
echo "    채널별 MPPI     채널당 0.31 ms 평균 / 0.65 ms 최대"
echo "    중앙집중 MPPI   0.45~0.55 ms 평균 (K=128), 스파이크 있음 → 데드라인이 막는다"
echo
if [ "$CORES" -lt 14 ]; then
  # 워커는 min(12, 코어)개이고 pin_cpus_[i] 로 CPU i 를 잡는다. 코어가 적으면 워커가
  # 모든 코어를 점유해 ROS 실행기·CAN 수신 스레드와 경합한다 — 그게 지터의 원인이 된다.
  KEEP=$(( CORES > 4 ? CORES - 2 : CORES - 1 ))
  [ "$KEEP" -lt 1 ] && KEEP=1
  PINS=$(seq -s, 0 $((KEEP - 1)))
  warn "워커 스레드가 모든 코어를 점유하면 ROS 실행기·CAN 수신과 경합한다."
  echo "     → 코어 2개를 남기도록 cpu_pins 를 줄이는 것을 권한다:"
  echo "         overrides:=cpu_pins=[${PINS}]      (yaml 을 고쳐도 된다)"
  echo "       또는 핀닝을 아예 끈다:  overrides:=enable_thread_pinning=false"
  echo "     → MPPI 표본 수도 먼저 낮춰 시작할 것:"
  echo "         overrides:=MPC_parameters.mppi_samples=64,MPC_parameters.sys_samples=64"
  echo "     → 그래도 '최대' 시간이 틱 예산의 60% 를 넘으면 period_ms 를 4(250 Hz)로."
else
  ok "기본 설정(cpu_pins 0..11, mppi_samples 128)으로 시작해도 된다"
fi

hdr "4. 빌드 산출물"
if [ -d "${WS}/install/can_powerpack/lib/can_powerpack" ]; then
  for exe in pp_controller virtual_powerpack; do
    [ -e "${WS}/install/can_powerpack/lib/can_powerpack/${exe}" ] \
      && ok "${exe}" || bad "${exe} 없음 — colcon build 필요"
  done
  if [ -e "${WS}/install/can_powerpack/lib/can_powerpack/can_bridge_node" ]; then
    ok "can_bridge_node (실기 필수)"
  else
    bad "can_bridge_node 없음 — Kvaser canlib 설치 후 재빌드할 것"
  fi
else
  warn "아직 빌드하지 않았다:"
  echo "     colcon build --packages-select qpoases_vendor can_powerpack \\"
  echo "                  --cmake-args -DCMAKE_BUILD_TYPE=Release"
  echo "     ('-march=native' 를 쓰므로 **이 기계에서** 빌드해야 한다. 바이너리 복사 불가)"
fi

hdr "5. 피팅 결과 반영 여부"
for f in valve_params.yaml pump_params.yaml; do
  if [ -f "${WS}/src/can_powerpack/config/${f}" ]; then
    ok "${f} 있음 — 기동 로그의 '밸브별 13-parameter: N/M' 로 로드 확인할 것"
  else
    warn "${f} 없음 — 피팅 전이면 정상 (하드코딩 기본값으로 돈다)"
  fi
done

hdr "5.5 엔코더 캘리브레이션 (액추에이터 붙이기 전 필수)"
# 실측 gain 은 세 보드 모두 음수인데 일반 기본값은 양수다 → 미측정 보드는 각도가
# 반대 방향으로 읽힌다. 위치 제어에서는 오차 부호가 뒤집혀 목표에서 멀어진다.
CFG="${WS}/src/can_powerpack/config/powerpack_config.yaml"
ENC="${WS}/src/can_powerpack/config/encoder_params.yaml"
missing=""
for bid in 17 18 19 20 21 22; do
  found=0
  for f in "$CFG" "$ENC"; do
    [ -f "$f" ] && grep -qE "\"${bid}\"[[:space:]]*:[[:space:]]*\{[^}]*raw_0deg" "$f" && found=1
  done
  [ "$found" -eq 0 ] && missing="${missing}${missing:+, }${bid}"
done
if [ -z "$missing" ]; then
  ok "board 17~22 모두 실측 2점 있음"
else
  bad "board ${missing} 실측값 없음 → 일반 기본값(gain **양수**)으로 돈다."
  echo "     실측된 보드는 gain 이 모두 **음수**다 → 그 축은 각도가 **반대 방향**으로 읽힌다."
  echo "     위치 제어 전에 캘리브레이션할 것 (RUNBOOK.md 0.5절):"
  echo "         ros2 run can_powerpack can_bridge_node --ros-args -r __ns:=/pack2 \\"
  echo "              --params-file src/can_powerpack/config/powerpack_config.yaml -p num_actuators:=6"
  echo "         python3 src/can_powerpack/scripts/encoder_calib.py --axes 0 1 2 3 4 5"
  echo "     무액추에이터 실험(4.5절 1단계)에는 영향 없다 — 각도를 쓰지 않는다."
fi

hdr "6. 남은 노드 (실기 전 반드시 비어 있어야 한다)"
N=$(pgrep -c -x 'pp_controller|can_bridge_node|virtual_powerpack' 2>/dev/null || true)
N=${N:-0}
if [ "$N" -eq 0 ]; then
  ok "제어기·브리지 프로세스 없음"
else
  bad "${N}개가 돌고 있다 — board/pwm_cmd 에 두 주인이 생긴다. 종료할 것:"
  pgrep -a -x 'pp_controller|can_bridge_node|virtual_powerpack' | sed 's/^/       /'
fi

hdr "다음"
echo "  RUNBOOK.md 4.5절 (실기 제어기 실험 — 순서와 스위치) 를 볼 것."
echo "  피팅부터 하려면 RUNBOOK.md 1·2절."
