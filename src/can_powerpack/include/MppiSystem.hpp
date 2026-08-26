#pragma once
// ============================================================================
// MppiSystem.hpp — **전체 시스템 중앙집중 MPPI**
//
// ── 왜 채널별에서 전체로 바꾸나 ─────────────────────────────────────────────
// 채널별 독립 MPPI(`Mppi.hpp`)는 공유 레일 압력을 지평 40 ms 동안 **상수**로 둔다.
// 실제로는 그렇지 않다:
//   · 레일 부피 500 mL vs 그 레일이 먹여 살리는 챔버 6개 합 300 mL → 비 1.67.
//     "무한 강성 소스" 가 아니라 부하와 비슷한 크기의 물통이다.
//   · 채널 밸브 **하나**의 완전 개방 유량(≈4.6 g/s)이 펌프 전체 토출(≲1 g/s)의 5배다.
//   · 계측: 40 ms 지평 동안 채널 1개 개방에 ≈5 kPa, 양압 6채널 동시에 **≈34 kPa** 강하.
//     추종 정규화 기준 10 kPa 대비 0.5~3.4σ 다.
//   · 양압 라인 PID 는 **대기 방출 밸브밖에 없어** 레일압을 올릴 수단이 없고 정상상태에
//     이미 포화(u=100)해 있다 — 채널이 레일을 끌어내릴 때 남은 권한이 0 이다.
//   · 그리고 "채널 수요 합 vs 레일 공급" 을 비교하는 코드가 저장소에 없었다.
//     12개 제어기가 서로를 모른 채 같은 500 mL 를 각자 무한 소스로 가정했다.
// 이것이 지평을 40 ms 이상 못 늘리는 원인이기도 했다 (80/120/160 ms 전부 악화).
//
// 전체 시스템을 **하나의 모델**로 굴리면 레일 강하와 유량 요구 총합이 자동으로 들어온다.
// 라인 밸브 2개도 같은 최적화 안에 들어오므로 라인 PID 가 하던 일을 흡수한다.
//
// ── 비용: 느리지 않다 ──────────────────────────────────────────────────────
// 플랜트의 밸브 개수는 어느 쪽이든 같다. 지금은 "채널 하나짜리 궤적" 을 12번 따로
// 샘플링하고, 중앙집중은 "전체 시스템 궤적" 을 한 번 샘플링한다.
//   현재    12채널 × 307 µs                  = 3.68 ms CPU/틱
//   중앙집중 K=128                            = 4.56 ms CPU/틱   (**1.24배**)
// 바뀌는 것은 병렬화 축이다: **채널 12개 → 표본 K개**. MPPI 표본은 완전 독립이라
// 더 잘 나뉜다. 24스레드에서 K=512 가 0.76 ms, K=1024 가 1.52 ms — 500 Hz 유지.
//
// ── 결정론 ────────────────────────────────────────────────────────────────
// 표본을 스레드에 나눠 돌리므로 RNG 를 순차로 돌리면 결과가 스케줄에 의존한다.
// **표본 인덱스로 시드**해서 어느 스레드가 어느 표본을 잡아도 같은 노이즈가 나오게 한다.
// 순차 실행과 병렬 실행이 비트 단위로 같아야 하고, 그것이 첫 번째 검증 테스트다.
//
// ── 1단계 범위 (회귀 위험 최소화) ──────────────────────────────────────────
// · 제어: 채널 micro 12 + 채널 atm 12 + 라인 밸브 2 = **26개**. macro 12개는 기존
//   피드포워드(compute_input_reference 의 유량 분배)에 남겨 차원을 줄인다.
// · 상태: 챔버압 12 + 레일압 2 + 밸브 상태. macro 라인(탱크·이젝터)은 조정되는
//   자원이고 40 ms 안에 거의 안 변하므로 **외생 상수**로 둔다.
// · 목적: 챔버 P_ref 12개 추종(기존과 동일) + **레일 밴드 유지**(라인 PID 역할 흡수)
//   + 노력·변화율. `TorquePID` 와 `PressureRefGen` 은 건드리지 않는다.
//
// ── 모델 식의 출처 ────────────────────────────────────────────────────────
// 레일·펌프 질량수지는 `VirtualPowerpack::integrate` (src/VirtualPowerpack.cpp:423-462)
// 와 **같은 식**이고 밸브 커널은 `Mppi.hpp` 를 그대로 재사용한다. 펌프는 ROS 무관
// 공용 헤더 `PistonPump.hpp` 의 능력 테이블을 쓴다.
// **바꿀 때는 반드시 양쪽을 함께 바꿀 것.** 어긋나면 예측이 시뮬과 다른 플랜트가 된다.
// 어긋남을 조용히 넘기지 않도록, 컨트롤러가 예측 레일압과 측정 레일압의 잔차를
// 주기적으로 로그에 찍는다 (연속 교차검증).
// ============================================================================

#include <cstdint>
#include <functional>
#include <vector>

#include "Mppi.hpp"
#include "PistonPump.hpp"

namespace mppi {

// ── 제어 벡터 배치 ─────────────────────────────────────────────────────────
//   [0 .. n_ch-1]        채널 micro (양압: 레일→챔버 / 음압: 챔버→레일)
//   [n_ch .. 2n_ch-1]    채널 atm   (양압: 챔버→대기 / 음압: 대기→챔버)
//   [2n_ch]              board1 v1  양압 레일 → 대기 (vent)
//   [2n_ch+1]            board2 v1  대기 → 음압 레일 (admit)
inline int sys_nu(int n_ch)      { return 2 * n_ch + 2; }
inline int sys_i_micro(int g)    { return g; }
inline int sys_i_atm(int n, int g) { return n + g; }
inline int sys_i_vent(int n)     { return 2 * n; }
inline int sys_i_admit(int n)    { return 2 * n + 1; }

struct SysParams {
  int n_ch{12};
  int n_pos{6};                        // 앞쪽 n_pos 개가 양압 채널

  std::vector<ChannelPlant> ch;        // 채널별 × 밸브별 13-parameter (n_ch 개)
  PlantParams line;                    // 라인 밸브 2개 공용 (별도 피팅 필요)

  // 레일
  float V_pos_m3{500e-6f}, V_neg_m3{500e-6f};
  float leak_pos{0.002f}, leak_neg{0.002f};      // [LPM/kPa]
  float pos_min{50.f},  pos_max{800.f};
  float neg_min{5.f},   neg_max{110.f};

  // 외생 (1단계에서는 지평 동안 상수)
  float P_atm{101.325f};
  float P_macro{801.325f};             // 탱크 레귤레이터 출력 (board 3)
  float P_macro_neg{11.325f};          // 이젝터 라인 (board 4)

  const pneu::PumpTable* pump{nullptr};   // 없으면 펌프 유량 0

  void finalize();                     // 채널·라인 PlantParams 사전계산
};

// ── 축 동역학 — **placeholder. 실측 후 채운다** ─────────────────────────────
//
// 지금 지평이 40 ms 로 묶여 있는 이유 중 하나는 챔버 부피 V 와 그 변화율 V̇ 를 지평
// 동안 상수로 두는 것이다. 실제로는 압력 → 힘 → 축 운동 → 부피 변화의 되먹임이 있고,
// 100 ms 를 넘는 지평에서는 그걸 예측해야 한다. 그리고 위치 계층 MPC(지평 300~500 ms)
// 는 이 모델 없이는 아예 만들 수 없다.
//
// **왜 지금 비활성인가**: 아래 값들이 실측되지 않았다. 시뮬 값
// (`config/virtual_powerpack.yaml` 의 `inertia_kgm2: 0.05`, `damping_nms: 0.30`) 은
// **시뮬 값이고 실측값이 아니다** — RUNBOOK.md 의 밸브·펌프 파라미터와 같은 처지다.
// 측정되지 않은 모델로 위치 계층을 최적화하면 MPPI 가 틀린 것을 잘 푸는 상태가 된다
// (그것이 지금 성능 한계의 원인이었다).
//
// ── 실측해야 하는 것 ──────────────────────────────────────────────────────
//   inertia_kgm2  : 축 관성. 무부하 스텝 응답의 2차 진동 주파수에서 역산하거나,
//                   알려진 토크 스텝의 초기 각가속도에서 J = τ/α 로 구한다.
//   damping_nms   : 점성 감쇠. 정속 구간의 τ 대비 ω 비율에서.
//   coulomb_nm    : 쿨롱 마찰. 움직이기 시작하는 최소 토크(브레이크어웨이).
//                   현재 TorquePID 의 friction_nm=0.48 은 손으로 맞춘 값이다.
//   tau_max_nm    : **단방향** 최대 토크. 이 액추에이터는 밀 수만 있고 당길 수 없다
//                   (`tau_ref = max(0, ...)`). 하강은 배기 + 부하에 의존한다.
//   reel_m        : 릴 반경 (25 mm, 이미 알려져 있다)
//
// 채워지면 열리는 것: (a) 위치 계층 MPC, (b) 지평 100 ms 이상, (c) 하강 궤적을
// 물리적으로 옳게 계획하는 것 (지금은 상승/하강 비대칭을 아무 계층도 모른다).
struct AxisDynamics {
  bool  enabled{false};        // 실측 전에는 false — 켜면 검증되지 않은 모델을 신뢰하게 된다
  float inertia_kgm2{0.05f};
  float damping_nms{0.30f};
  float coulomb_nm{0.48f};
  float tau_max_nm{7.76f};
  float reel_m{0.025f};
  float mass_kg{5.0f};
  float link_length_m{0.15f};
};

// 축 상태 한 스텝 전진 — **미구현**. `ad.enabled` 가 false 면 아무 것도 하지 않는다.
// 구현할 때 필요한 식 (VirtualPowerpack.cpp:520~ 의 회전 동역학과 같아야 한다):
//   J·ω̇ = τ_net − damping·ω − sign(ω)·coulomb − m·g·L·sin(θ)
//   τ_net = reel · (P⁺·A⁺ − P⁻·A⁻),  단 τ_net ≥ 0 (단방향)
//   θ̇ = ω,  그리고 θ → 챔버 부피 (Controller.cpp:1654-1673 의 각도-부피 식)
void axis_step_placeholder(const AxisDynamics& ad, float& theta_deg, float& omega_dps,
                           float tau_net_nm, float dt);

struct SysState {
  std::vector<float> P_ch;             // [n_ch] 챔버 절대압 [kPa]
  float P_pos{155.f}, P_neg{30.f};     // 레일 절대압 [kPa]
  std::vector<ValveState> v;           // [n_ch*3 + 2]  (채널 3개씩 + vent + admit)

  void resize(int n_ch);
  int  iv_ch(int g, int j) const { return g * 3 + j; }
  int  iv_vent()          const { return (int)v.size() - 2; }
  int  iv_admit()         const { return (int)v.size() - 1; }
};

// 지평 동안 채널별로 주어지는 외생 입력 (액추에이터 운동에서 온다)
struct SysExo {
  std::vector<float> V, Vdot;          // [n_ch] 챔버 부피 [m³], 변화율 [m³/s]
  std::vector<float> P_ref;            // [n_ch] 챔버 목표 절대압 [kPa]
  std::vector<float> u_macro;          // [n_ch] macro 밸브 지령 [%] — 최적화 대상 아님
  float rail_pos_sp{155.f}, rail_neg_sp{30.f};
  // control_lines=false 일 때 라인 밸브 개도를 **외생 입력**으로 받는다 (LinePID 출력).
  float u_vent{0.f}, u_admit{0.f};
  float ref_tau_s{0.12f};              // 스테이지 레퍼런스 접근 시상수 (Mppi.hpp 3.6(d))
};

// 한 스텝 전진. u 는 **테이퍼·클램프까지 끝난 인가 명령** [%] (길이 sys_nu).
void sys_step(const SysParams& p, SysState& s, const float* u,
              const SysExo& ex, float dt);

// ── MPPI 하이퍼파라미터 ────────────────────────────────────────────────────
struct SysMppiParams {
  int   K{256};
  int   NP{10};
  float Ts{0.004f};
  int   substeps{1};

  float lambda{0.15f};             // λ_eff = λ·(median(J) − min(J))
  float sigma_pct{8.0f};
  float sigma_explore_pct{30.0f};
  float explore_frac{0.30f};
  float noise_beta{0.70f};

  float w_track{1.0f};             // 챔버 추종
  float w_rail{0.5f};              // 레일 밴드 유지 (라인 PID 역할)
  float w_effort{0.10f};
  float w_du{0.05f};
  float track_scale_kpa{10.0f};
  float rail_scale_kpa{20.0f};
  float terminal_mult{0.0f};

  // ── 공로 배분 (factored MPPI) ────────────────────────────────────────────
  // 표본 하나가 26개 밸브를 동시에 흔들면, 채널 3 에 도움이 되고 채널 7 에 해로운
  // 표본이 중간 점수를 받아 가중 평균이 전부를 뭉갠다 (계측: IAE 6 → 30).
  // 그래서 **예측은 중앙집중, 가중 평균은 채널별**로 나눈다. 롤아웃은 전체 시스템으로
  // 굴려 레일 결합을 얻고, 비용·가중은 그룹(채널 12개 + 라인 1개)마다 따로 계산한다.
  //
  // rail_share: 레일 비용 중 각 채널에 배분하는 비율. 채널이 레일을 끌어내리는 것에
  // 대한 **공유 가격**이다 (분산 최적화의 dual price 와 같은 역할). 0 이면 채널은
  // 레일을 전혀 신경쓰지 않고, 크면 레일을 지키려 챔버 추종을 희생한다.
  float rail_share{0.20f};

  // 라인 밸브(vent/admit)를 MPPI 가 **직접 지령**할지.
  //
  // false(기본) — 레일은 **예측만** 하고 제어는 LinePID 에 맡긴다. 레일 예측이
  //   챔버 예측을 정확하게 만드는 것이 중앙집중의 핵심 이득이고, 그것만 취한다.
  // true — MPPI 가 라인 밸브까지 소유한다. 계측 결과 **불안정했다**: 양압 레일이
  //   74↔800 kPa 를 오가고 음압이 대기압(101)을 넘는 107 까지 갔다 (LinePID 는
  //   222/100, 28/74). 레일 동역학이 채널 하나에 2479 kPa/s 로 매우 빠른데 2 ms 마다
  //   재계획하니 뱅뱅 동작이 된다. 안정화에는 레일 전용 변화율 벌점·더 긴 라인 밸브
  //   유지 시간 같은 추가 설계가 필요하다 (MPPI.md 참조).
  bool  control_lines{false};

  float du_limit_pct{100.f};
  float flat_spread{1.0e-3f};
  float flat_decay{0.90f};
};

struct SysStats {
  uint64_t calls{0}, flat{0}, sat_first{0};
  double sum_us{0.0}, sum_eff{0.0}, sum_cost{0.0}, sum_spread{0.0};
  float  max_us{0.f};
};

// 표본 병렬화 훅. 호출자가 ThreadPool 을 넘긴다 (Mppi 를 ROS·스레드 무관으로 유지).
// 순차 구현을 넘기면 결정론 검증용 기준이 된다.
using ParallelFor = std::function<void(int n, const std::function<void(int)>&)>;

class SystemSolver {
public:
  SystemSolver(const SysParams& sp, const SysMppiParams& mp, uint32_t seed);

  // uref  : 피드포워드 명령 [%] (길이 sys_nu). 채널 부분은 역모델+적분항, 라인 2개는
  //         기존 LinePID 출력을 넘기면 그것을 명목으로 삼는다 (안전한 초기 추정).
  // 반환  : 첫 스텝 보정 Δu [%] (길이 sys_nu)
  const std::vector<float>& solve(const SysState& x0, const SysExo& ex,
                                  const std::vector<float>& uref,
                                  const ParallelFor& pfor);

  void reset();
  SysStats take_stats();
  const SysMppiParams& params() const { return mp_; }
  // 예측 검증용: 직전 solve 의 명목 궤적 끝 레일압 (측정과 비교해 모델 어긋남을 본다)
  // **1스텝 앞** 예측. 40 ms 앞 예측을 현재 측정과 비교하면 당연히 크게 벌어진다
  // (그 실수로 레일 예측오차가 22.8 kPa 로 보였다). 다음 틱 측정과 비교할 것.
  float pred1_rail_pos() const { return pred1_pos_; }
  float pred1_rail_neg() const { return pred1_neg_; }
  bool  pred1_valid()    const { return pred1_valid_; }

private:
  float rollout(const SysState& x0, const SysExo& ex,
                const std::vector<float>& uref, int sample);

  SysParams     sp_;
  SysMppiParams mp_;
  uint32_t      seed_{0};
  uint64_t      call_{0};
  SysStats      st_{};

  int nu_{0}, nseq_{0};
  std::vector<float> nom_, dnom_, du0_, cost_, w_, sortbuf_;
  std::vector<float> noise_;          // [K * nseq]
  std::vector<Rng>   rng_;            // [K] 표본별 — 스레드 무관 결정론
  // 표본별 작업공간 — **전부 생성 시 예약한다.** 롤아웃 안에서 vector 를 만들면
  // 표본당 4회 × K=256 = 틱당 1024회 할당이 12스레드에서 동시에 일어나 malloc 경합이
  // 계산을 압도한다 (계측: 평균 1138 us, 최대 16416 us — 틱 2 ms 예산의 8배).
  std::vector<std::vector<float>> eps_;      // [K][nseq] 노이즈
  std::vector<std::vector<float>> sc_prev_;  // [K][nu]   상관 노이즈 이전값
  std::vector<std::vector<float>> sc_gap_;   // [K][n_ch] 스테이지 레퍼런스 잔여
  std::vector<std::vector<float>> sc_uapp_;  // [K][nu]   인가 명령
  std::vector<std::vector<float>> sc_uprv_;  // [K][nu]   직전 스텝 명령
  std::vector<std::vector<float>> sc_jg_;    // [K][n_ch+1] 그룹별 비용
  std::vector<SysState> work_;               // [K] 롤아웃 상태
  std::vector<float> gcost_, gsort_, gw_;    // [K] 그룹 처리용
  std::vector<float> gacc_;                  // [2*NP] 그룹 가중 노이즈 누적
  std::vector<float> u_full_;                // [sys_nu] 라인 미제어 모드용 결합 버퍼
  float u_line_[2]{0.f, 0.f};
  int n_grp_{0};
  // 1스텝 예측 (진단) — 다음 틱 측정과 비교해야 의미가 있다.
  float pred1_pos_{155.f}, pred1_neg_{30.f};
  bool  pred1_valid_{false};
};

}  // namespace mppi
