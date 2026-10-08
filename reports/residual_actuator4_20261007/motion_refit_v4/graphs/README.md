# 재피팅 v4 그래프

검정: 실측. 주황 점선: 공칭. 파랑: 공칭 + passive residual.

- `01_all_runs_overlay.png`: 전체 유효 표본의 누적 시간 그래프.
- `02_profiles_overview.png`: 12개 실행 비교.
- `03_...png`–`14_...png`: 실행별 각도·오차·측정 압력.
- `comparison_all_pages.pdf`: 위 14개 그림을 수록한 PDF.
- `15_before_after.png` / `.pdf`: 직전 v3와 이번 v4의 대표 5개 실행 비교.
- `16_before_after_zoom.png` / `.pdf`: 대표 구간 확대, 좌우 같은 각도 축.

전후 비교 그림의 왼쪽은 v3, 오른쪽은 v4다. 확대 그림 RMSE는 표시 구간에
한정한 값이다. 모든 예측은 같은 측정 압력·같은 시작 각도/속도를 사용했다.
블록/실행 경계를 선으로 잇지 않는다. 선택 모델과 학습·개발·기존 진단의
구분, 개선과 악화 사항은 상위 `REPORT.md`를 참고한다.

재생성:

```sh
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/ph_mpl python3 -m ph_model.plot_predictions \
  --csv reports/residual_actuator4_20261007/motion_refit_v4/all_runs.csv
PYTHONNOUSERSITE=1 PYTHONPATH=.residual_deps MPLCONFIGDIR=/tmp/ph_mpl \
  python3 -m ph_model.plot_motion_comparison
```
