# 공칭 재식별: 중력·왕복 분리·반경 민감도

결과는 `reports/residual_actuator4_20261007/nominal_refit_v3/`에 별도로 저장한다.
ROS·압력 제어 코드·TCP는 사용하지 않고 원본 로그를 읽기만 한다.

## 공칭 단계

1. 학습 S2는 중심압별, S1a/S1b는 챔버별로 상승/하강 정착점을 같은 각도에서 짝짓는다.
   두 방향이 겹치는 각도 범위에서만 보간하며 외삽하지 않는다. 중복 각도는 평균한다.
   101.3과 101.30000000000001처럼 부동소수 표현만 다른 중심압은 같은 그룹이다.
2. 왕복 평균 압력의 nominal 토크와 `Mg*sin(q)+k*q+c`를 맞춘다.
   왕복 대칭 근사를 사용한 탐색적 중심 특성이지 실제 토크를 측정한 것은 아니다.
3. 고정 반경별 Mg/k/c/x1을 3개 시작점에서 피팅한다. 비교 손실은 Nm가 아니라
   `토크 오차 / 반경`인 힘[N]으로 정규화해 작은 반경 자체에 보상을 주지 않는다.
4. 같은 각도 왕복 토크 반폭으로 alpha 초기값을 잡는다. 이는 초기 근사일 뿐
   동역학에서 tanh를 접근 방향의 sign으로 바꾸지 않는다.
5. 25 mm에서는 중심 특성을 고정한 alpha/b 장시간 피팅을 별도 수행한다.
6. 20/22.5/25/27.5/30 mm 각각에서 Mg/k/c/alpha/b를 전체 학습 궤적에 재피팅한다.
   중심 힘 오차 anchor도 둔다. J, D, L0, n, epsilon은 고정한다.
7. 숫자상 개발 최선 후보와 물리적으로 식별된 파라미터를 구분한다.
   탐색 하한에 몰린 Mg를 실제 질량/부하 거리로 환산해 확정하지 않는다.

Mg 탐색 범위 0.2–8 Nm, k 0–10 Nm/rad, c −3–3 Nm, x1 50–150 mm는
민감도 검사용 범위이지 실측 허용오차가 아니다. Mg와 탄성이 서로 대체되는지
별도의 조건부 Mg 프로파일도 계산한다. 질량과 레버암을 각각 식별하지 않는다.

반경이 바뀌면 `x1=x10-r*q`, `x2=x20+r*q`, 토크, 압력 포트의 r이 모두 함께 바뀐다.
따라서 전력 항등식은 유지된다. 반경에 따른 NN 정규화 변화를 이용해 데이터를
몰래 재해석하지 않는다. 반경 스캔에서는 residual=0인 공칭만 사용한다.

## Residual 단계

물리적으로 의심스러운 중력 하한의 수치 최적해를 승격하지 않고,
25 mm 왕복 중심 피팅 + alpha/b 피팅 공칭을 고정한 별도 residual을 학습한다.
이 공칭도 고유한 실제 물리값으로 식별됐다는 뜻은 아니다.

퍼텐셜·소산 NN은 새 float64 SiLU 네트워크의 출력 0에서 시작한다.
식 오차 600스텝은 초기화에만 사용하고, 이후 두 양의 이력 초기값에서
전체 유효 학습 블록 자유 예측을 최적화한다. 실제 각도·속도는 시작에만 쓰며
이후 내부 이력은 모델 속도로 진행한다.

학습 변수는 퍼텐셜 출력층, 비음수 마찰 혼합, 양의 이력 k/rho다.
nominal, NN 은닉층, rv, 추가 점성 항은 장시간 최적화 중 고정한다.
압력 의존 저장 에너지·자유 힘 NN·자유 RNN·ReLU는 추가하지 않는다.
학습된 면적 보정과 다개체 코드는 이번 실험에 포함하지 않는다.

개발 seed 3의 독립 적응 적분 RMSE로 이전 v2까지 포함해 후보를 비교한다.
기존 seed 4 정상 prefix는 이미 진단한 데이터이므로 선택 이후 비교만 하며
독립 블라인드 검증으로 표현하지 않는다. 기존 결과와 USB를 덮어쓰지 않는다.

```sh
PYTHONNOUSERSITE=1 PYTHONPATH=.residual_deps MPLCONFIGDIR=/tmp/ph_mpl \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m ph_model.refit_nominal

PYTHONNOUSERSITE=1 PYTHONPATH=.residual_deps MPLCONFIGDIR=/tmp/ph_mpl \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m ph_model.retrain_physical_residual
```

공칭 후보 수치는 `nominal_candidates.json` / `radius_profile.json` / `gravity_profile.json`,
residual 결과는 `residual/candidates.json` / `residual/selection.json` / `residual/metrics.json`에 있다.
공칭의 수치상 최선과 실제 물리값으로의 채택 가능성을 혼동하지 않는다.
