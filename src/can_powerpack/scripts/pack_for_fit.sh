#!/usr/bin/env bash
# 피팅용 데이터 묶음 — 계산이 느린 기계에서 빠른 기계로 옮길 때 쓴다.
#
# 코드는 git 으로 간다 (원격에 있다). 여기서 싸는 것은 **원본 CSV 뿐**이다 —
# results_fit/ 는 .gitignore 대상이라 git 으로 못 옮긴다.
#
# 사용:  scripts/pack_for_fit.sh [출력경로.tar.gz] [디렉터리...]
#        기본: refit_pos_micro, refit_pos_macro
set -euo pipefail
cd "$(dirname "$0")/.."

OUT="${1:-/tmp/valve_fit_data_$(date +%Y%m%d_%H%M).tar.gz}"
shift 2>/dev/null || true
DIRS=("$@")
if [ ${#DIRS[@]} -eq 0 ]; then
  DIRS=(results_fit/refit_pos_micro results_fit/refit_pos_macro)
fi

echo "== 묶을 대상 =="
KEEP=()
for d in "${DIRS[@]}"; do
  if [ ! -d "$d" ]; then echo "  건너뜀 (없음): $d"; continue; fi
  n=$(find "$d" -name '*.csv' -size +1k | wc -l)
  z=$(find "$d" -name '*.csv' -size -1k | wc -l)
  echo "  $d — CSV ${n}개$([ "$z" -gt 0 ] && echo " (빈 파일 ${z}개는 제외)")"
  KEEP+=("$d")
done
[ ${#KEEP[@]} -gt 0 ] || { echo "묶을 것이 없다"; exit 1; }

# 빈 CSV(헤더만 있는 실패 회차)와 플롯·중간산출물은 뺀다 — 데이터만 옮긴다.
tar czf "$OUT" \
    --exclude='*.png' --exclude='report.md' --exclude='valve_params.yaml' \
    --exclude='*.log' \
    $(find "${KEEP[@]}" -name '*.csv' -size +1k -printf '%p\n' | sed 's/\.csv$//' \
      | while read -r s; do printf '%s.csv %s.meta.yaml ' "$s" "$s"; done)

echo
echo "== 완료 =="
ls -lh "$OUT"
echo
echo "다음 기계에서:"
echo "  1) git pull                       # 코드"
echo "  2) tar xzf $(basename "$OUT") -C <레포>/src/can_powerpack/"
echo "  3) pip install numpy pyyaml matplotlib     # ROS 불필요"
echo "  4) scripts/ 안내대로 valve_fit_solve.py 실행"
