# 논문 nominal 모델의 오프라인 피팅

`main.tex`의 `eq:kin`, `eq:Am`, `eq:tau`, `eq:fric`, `eq:ph0`를 구현한다.
ROS·TCP·qpOASES·실기 제어기 없이 동작한다. residual/NN/잠재 이력 상태는 없다.
이 패키지의 시뮬레이션은 모델 예측 평가이며, 실기 제어 코드의 검증이 아니다.

## 재현

워크스페이스 루트에서 실행한다. 기존 사용자 Python에는 NumPy 1.26.4와
SciPy 1.8.0의 지원 버전 불일치가 있어, 설치를 변경하지 않고 시스템 패키지
(NumPy 1.21.5, SciPy 1.8.0)를 사용한다.

```sh
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/ph_mpl OPENBLAS_NUM_THREADS=1 \
  python3 -m pytest -q tests/test_ph_nominal.py tests/test_ph_data.py

PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/ph_mpl OPENBLAS_NUM_THREADS=1 \
  python3 -m ph_model.fit_nominal

PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/ph_mpl OPENBLAS_NUM_THREADS=1 \
  python3 -m ph_model.refine_nominal

PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/ph_mpl OPENBLAS_NUM_THREADS=1 \
  python3 -m ph_model.audit_fit
```

입력 기본값은 `/home/risebrl/result/ph/4`이며 읽기만 한다.
출력 기본값은 `reports/nominal_actuator4_20261006`이다. 재실행 시 해당 파생
결과를 갱신하므로 별도 실험은 `--output`으로 다른 폴더를 지정한다.
`fit_nominal` 결과가 있는 폴더를 `refine_nominal`에 넘겨야 한다.

## 단위와 인터페이스

- `Parameters.load()`가 `params.yaml`의 기준값을 읽는다.
- `torque(q, pressure, p)`: `q` rad, `pressure[..., :] = [P1, P2]` Pa gauge,
  반환 Nm. `P1 = (Pneg_abs − 101.325)*1000`, `P2 = (Ppos_abs − 101.325)*1000`.
- `rhs([q, qdot], pressure, p)`: 외부 토크를 0으로 둔 오프라인 운동방정식.
- `energy(q, qdot, p)`: `J*qdot²/2 + V0(q)`. 논문의 운동량 `p=J*qdot`와 동치다.
- `x2_zero_m`는 상수 양압 면적 때문에 동역학에 나타나지 않는다. 추정하지 않는다.
- 기하 영역을 벗어나면 오류로 중단한다. 측정 압력의 부호는 로더에서 마스크로
  구분하며, 데이터나 입력을 클리핑하지 않는다.

## 해석 주의

`nominal_fitted.yaml`은 무압 탄성을 0으로 둔 제한된 기준 모델이다.
`elastic_equation.yaml`, `elastic_rollout.yaml`은 `Ve=k*q²/2+c*q`를 추가 가정한
nominal 후보이며, 독립 무압 탄성 측정으로 확인된 값이 아니다. 임의 힘 NN은 아니다.
관성 0.045 kg·m²는 끝단 점질량만 반영한 가정이고 진자 실측값이 아니다.

정착점의 접근 방향으로 Coulomb 부호를 정한 근사는 초기 진단용으로만 남긴다.
실제 예측은 논문의 `tanh(qdot/epsilon)` 식을 그대로 적분한다. 정지 상태에서
마찰이 사라지는 식을 정마찰 모델로 바꾸지 않는다.

`refine_nominal`의 짧은 창 최적화에는 배치 implicit midpoint를 사용하지만,
최종 오차는 독립 구현한 adaptive LSODA 장시간 자유 예측으로 계산한다.
비선형 퍼텐셜에 일반 implicit midpoint를 쓴 것만으로 이산 수동성이 자동 보장되지는
않는다. 연속시간 전력 항등식·소산 조건과 수치 적분 오차를 구분한다.

결과와 데이터 분할, 가정, 파라미터 경계, 논문에서 보완할 문장은 결과 폴더의
`REPORT.md`를 먼저 읽는다. 이 결과를 검증된 실기 제어 파라미터로 배포하지 않는다.
