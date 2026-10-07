# 긴 자유 예측 기반 residual 보완

기존 최종 모델 `reports/residual_actuator4_20261007/best_model.json`을 기준판으로
보존하고 `long_rollout_v2/`에 별도 결과를 만든다. ROS·TCP·하드웨어를 사용하지 않는다.

## 바뀌는 것과 고정하는 것

- 학습 목적: 관측 미분으로 만든 토크 식 오차가 아니라 전체 유효 학습 블록의 각도 오차.
  10초마다 각도를 재주입하지 않는다. 시작 각도·속도만 기존 전처리와 동일하게 사용한다.
- 이력 상태는 학습 중에도 모델 속도로 적분한다. 관측 속도의 잡음으로 이력 관측기를
  계속 구동하는 기존 학습/예측 불일치를 피한다. 원본 각도나 필터를 조용히 수정하지 않는다.
- 학습 변수: 기존 NN 퍼텐셜 출력층 17개, 비음수 마찰 혼합 계수 4개,
  양의 이력 강성 2개와 양의 속도 의존 이완 계수 2개 (총 25개).
- 고정: 모든 nominal 값(J·기하·중력·마찰·epsilon 포함), NN 은닉 기저,
  추가 점성 계수, 이력 정지 이완율 rv, 유효면적. 기준 학습 모델에서 재시작한다.
- V는 기존과 같은 압력 독립 스칼라 `2*tanh(raw/2)`이고 힘은 그 미분이다.
  자유 힘 NN·압력 의존 저장 에너지·자유 RNN·ReLU를 추가하지 않는다.
- H_h는 `sum(k*(q-xi)^2/2)`, `xi_dot=(rv+rho*abs(v))*k*(q-xi)` 그대로다.
  k,rho는 로그 좌표로 최적화해 양수를 보장하고 마찰 계수는 비음수 경계를 둔다.
  rv는 기존 식별 하한값으로 고정하므로 크리프 시정수를 새로 식별했다고 주장하지 않는다.

학습 데이터 분할은 기존과 같다. 프로파일별 총 가중치는 같고 각도 손실은 2 Hz로
계산하지만, 압력 입력은 10 Hz, 적분 간격은 50 ms다. 구간 사이 결측을 잇지 않는다.
미리 지정한 두 초기값을 각각 최적화하고 **개발 seed 3의 독립 적응 적분 RMSE**로
기준판 포함 최종 후보를 선택한다. 기존 seed 4 prefix는 이미 진단한 데이터이므로
새 블라인드 시험이라고 부르지 않는다. 후보 선택 후 비교만 하고 추가 튜닝하지 않는다.

## 가속 계산과 검증

`residual_fast.cpp`는 기존 float64 pH 식을 implicit midpoint로 계산하는 오프라인
최적화 가속기다. Python이 `g++`로 임시 폴더에 공유 라이브러리를 빌드한다.
수치 실패·기하 영역 이탈을 검출하며 압력·각도를 클리핑하지 않는다.
테스트는 모든 상태 q,v,xi를 별도 SciPy 적응 적분기와 비교한다.
최종 보고 수치는 기존 SciPy 적분기로 다시 계산한다.

연속시간 수동성은 기존 에너지 식에서 유지된다. 일반 비선형 H에 대한
이산 스텝별 수동성을 midpoint만으로 보장한다고 주장하지 않는다.
최종 후보의 전력 항등식·전체 에너지 수지·무입력 에너지와 적분 수렴을 따로 검사한다.

```sh
PYTHONNOUSERSITE=1 PYTHONPATH=.residual_deps MPLCONFIGDIR=/tmp/ph_mpl \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m pytest -q tests

PYTHONNOUSERSITE=1 PYTHONPATH=.residual_deps MPLCONFIGDIR=/tmp/ph_mpl \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m ph_model.improve_residual

PYTHONNOUSERSITE=1 PYTHONPATH=.residual_deps MPLCONFIGDIR=/tmp/ph_mpl \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m ph_model.audit_long_residual
```

`long_rollout_v2/selected_model.json`에 후보를 저장한다. `comparison.json`에는 이전
nominal·residual과의 전체 비교가, `all_runs.csv`에는 연속 표시 시간·측정 압력·실제 각도·
새 예측·이전 nominal·이전 residual 예측이 들어간다. 기존 CSV와 USB 파일은 변경하지 않는다.
