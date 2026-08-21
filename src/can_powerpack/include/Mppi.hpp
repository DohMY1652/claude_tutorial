#pragma once
// ============================================================================
// Mppi.hpp — 채널 MPC 의 샘플링 기반 솔버 (Model Predictive Path Integral)
//
// ── 왜 선형화를 버렸나 ──────────────────────────────────────────────────────
// 기존 경로(update_linearization → build_mpc_qp → solve_qp_first_step)는
// 13-parameter 밸브 모델을 수치 야코비안으로 선형화해 (A_scalar, B_row) 를 만들고
// Δu 에 대한 응축 QP 를 qpOASES 로 풀었다. 두 가지가 **구조적으로** 깨져 있었다:
//
//   1) 초킹 구간에서 A_scalar 가 정확히 0 이다. 초킹 유량은 하류압과 무관하므로
//      dQ/dPout = 0 이고, 양압 채널의 A_scalar = mi[2] + ma[2] − at[1] − lk[1] 가
//      통째로 0 이 된다. 그러면 T_bar = 0 이 되어 목적함수가 (P_ref − P_now) 가 아니라
//      **절대압 Xref** 를 추종하려 한다. 동작점 대부분이 초킹이라 상시 조건이다.
//   2) A_scalar 는 연속시간 야코비안 ∂Ṗ/∂P [1/s] 인데 build_mpc_qp 가 그것을 이산
//      전이행렬로 써서 A¹..A^NP 를 만든다 (1 + A·Ts 여야 한다).
//
// 게다가 이 플랜트의 핵심 비선형성은 선형화로 표현될 수 있는 종류가 아니다:
//   · 크래킹 임계 — 명령 ≈52%(상류압 의존) 아래에서 유량이 정확히 0 이다.
//     A_eff = A_max·sigmoid(k·F)^alpha 에서 alpha≈3884 이므로 사실상 계단 함수다.
//   · 초킹 — 하류압 민감도가 구간에 따라 0 이거나 유한하다.
//   · 명령 테이퍼 — 목표 근처에서 "크래킹 임계 위쪽 여유분"만 줄인다.
//   · 밸브 2차 동특성(wn≈40 rad/s → τ≈25 ms)이 **지평(NP·Ts=40 ms)과 같은 스케일**인데
//     선형화 경로는 이걸 아예 무시했다. 지평 안에서 가장 지배적인 동특성이다.
//
// MPPI 는 기울기가 필요 없다. 위 모델을 그대로 K 개 롤아웃하고 exp(−J/λ) 가중 평균한다.
// 계단·불연속·포화·상태 의존 지연이 전부 자연스럽게 들어간다.
//
// ── 무엇을 대체하고 무엇을 남겼나 ──────────────────────────────────────────
// 대체: update_linearization → build_mpc_qp → solve_qp_first_step  (Δu 산출)
// 유지: 역모델 피드포워드 + 적분항(uref), 명령 테이퍼, 크래킹 클램프, macro 게이트
//
// MPPI 는 QP 가 있던 **같은 자리**에서 uref 주변의 보정 Δu 를 낸다. 피드포워드를 없애고
// MPPI 에 전권을 주는 것도 가능하지만, 밸브 13-parameter 가 아직 전부 추정값이라
// (README 0절) 정상상태 오차를 밀어내는 적분항이 필요하다. 파라미터 피팅이 끝나면
// 전권 MPPI 로 넘어가는 것이 다음 단계다.
//
// ── 롤아웃 모델은 VirtualPowerpack 과 식이 동일하다 ─────────────────────────
// `VirtualPowerpack::step_valve` / `pressure_derivative` 와 같은 식·같은 상수
// (R=287, T=293.15 K, LPM→kg/s = 2.155e-4, κ=1.4)·같은 오일러 순서를 쓴다.
// 그래야 가상 하드웨어에서의 비교가 "모델이 달라서 생긴 차이"가 아니게 된다.
// ============================================================================

#include <array>
#include <cstdint>
#include <vector>

namespace mppi {

// VirtualPowerpack.hpp 와 동일 — 의도적으로 중복시켰다. 여기서 값이 갈리면
// 롤아웃이 시뮬과 다른 플랜트를 예측하게 되므로, 바꿀 때는 반드시 양쪽을 함께 바꾼다.
constexpr float RGAS_AIR    = 287.0f;      // [J/(kg·K)]
constexpr float TEMP_K      = 293.15f;     // [K]
constexpr float LPM_TO_KGPS = 0.0002155f;  // [LPM] → [kg/s] (경험 상수)
constexpr float KAPPA       = 1.4f;        // 공기 비열비

// 밸브 인덱스 — AcadosMpc::last_u3_ / u_crack_ 와 동일 순서
enum ValveIdx : int { V_MICRO = 0, V_MACRO = 1, V_ATM = 2 };

// ── 13-parameter 밸브 + 채널 구성 (AcadosMpc::Config 에서 복사) ─────────────
struct PlantParams {
  float I_MAX{0.30f}, A_max{0.2845f}, k_shape{33.09f};
  float C_k{0.0288f}, C_p{0.00012f}, C_z{0.0f}, alpha_shape{3884.2f};
  float A_bw{260649.5f}, beta_bw{179.0f}, gamma_bw{0.06f};
  float wn_up{40.0f}, zeta_up{1.2f}, wn_down{45.0f}, zeta_down{1.0f};

  bool  is_positive{true};
  float ejector_p_limit{11.325f};
  float leakage_u{0.0f};           // 이 채널 방향의 누설 등가 명령 [%]
  float crack_area_frac{1e-6f};    // "닫힘"으로 볼 A_eff/A_max
  float cmd_taper_kpa{3.0f};

  // ── finalize() 가 채우는 사전계산값 ──────────────────────────────────────
  // F_crack: A_eff = crack_area_frac·A_max 가 되는 F_net. sigma 가 crack_area_frac 만으로
  //   정해지므로 **Pin·z 와 무관한 상수**다 → 런타임에 log 를 부를 필요가 없다.
  //   u_crack(Pin,z) = (F_crack − C_z·z − C_p·Pin + C_k)/I_MAX·100  (Pin·z 에 affine)
  // F_open: 이 F_net 이하이면 A_eff 가 float 0 으로 언더플로한다 → exp/log1p 조기 탈출.
  float F_crack{0.0f};
  float F_open{0.0f};
  void finalize();

  // 크래킹 임계 [%] — 이 명령 이하에서는 스풀이 들리지 않아 유량이 0 이다.
  float u_crack(float Pin, float z) const;
};

// ── 밸브 1개의 상태 ────────────────────────────────────────────────────────
// q/qd: 2차 동특성이 걸린 유량과 그 미분 [LPM], [LPM/s]
// z/prevI/dir: Bouc-Wen 히스테리시스
struct ValveState {
  float q{0.f}, qd{0.f};
  float z{0.f}, prevI{0.f};
  int   dir{1};
};

struct ChannelState {
  float P{101.325f};                 // 챔버압 [kPa abs]
  std::array<ValveState, 3> v{};
};

// 지평 동안 상수로 두는 외생 입력. 레일압은 다른 계층(생성기·라인 PID)이 정하고
// 40 ms 지평 안에서는 거의 안 변하므로 상수로 두는 것이 타당하다.
struct Exogenous {
  float P_micro{101.325f};   // 양압: 양압레일 / 음압: 음압레일
  float P_macro{101.325f};   // 양압: 탱크    / 음압: 미사용(이젝터는 p_limit)
  float P_atm{101.325f};
  float V0{1.0e-5f};         // 챔버 부피 [m³]
  float Vdot{0.f};           // 부피 변화율 [m³/s] — 축이 움직이면 압력이 변한다
  float P_ref{101.325f};
  // 지평 안의 스테이지 레퍼런스를 1차 접근 궤적으로 만든다:
  //   P_ref(k) = P0 + (P_ref − P0)·(1 − exp(−k·Ts/tau_ref))
  //
  // 왜: 지평 전체를 P_ref 상수로 두면 "40 ms 안에 전 오차를 닫아라" 가 된다. 그런데
  // P_ref 는 PressureRefGen 이 **슬루 제한해 만든 궤적의 현재 점**이고, 챔버 50 mL 에
  // 밸브를 열면 4 ms 에 99 kPa 가 움직이므로 MPPI 는 실제로 그걸 닫을 수 있다 —
  // 그래서 최대 공격성으로 밀고, 위치가 오버슈트한다 (계측: 오버슈트 4.7~8.7° vs QP 3.0°).
  // 피드포워드가 이미 target_time_constant 로 같은 접근 궤적을 겨냥하고 있으므로,
  // MPPI 도 같은 궤적을 기준으로 삼으면 "경쟁하는 공격적 제어기" 가 아니라
  // "그 궤적에서 모델 오차를 메우는 보정기" 가 된다.
  float tau_ref{0.5f};       // 접근 시상수 [s]. ≤0 이면 상수 레퍼런스.
  float P0{101.325f};        // 지평 시작 압력 (스테이지 레퍼런스 생성 기준)
};

// ── 모델 커널 (테스트·재사용을 위해 공개) ──────────────────────────────────
float phi(float Pin, float Pout);                                  // 압축성 유동 Φ
float area_eff(const PlantParams& p, float u_pct, float Pin, float z);
float q_static(const PlantParams& p, float u_pct, float Pin, float Pout, float z);
float step_bw(const PlantParams& p, ValveState& vs, float u_pct);   // z 갱신 후 z 반환
float valve_dyn(const PlantParams& p, ValveState& vs, float Q_static, float dt);
// 한 스텝 전진. u 는 **테이퍼·클램프까지 끝난 실제 인가 명령** [%] 3개.
void  step(const PlantParams& p, ChannelState& s, const std::array<float, 3>& u,
           const Exogenous& ex, float V, float dt);

// 밸브 내부 상태 추정(q, qd)만 **측정 압력**으로 한 스텝 전진시킨다.
//
// 왜 필요한가: 2차 동특성의 τ≈25 ms 가 지평 40 ms 와 같은 스케일이라, 롤아웃을
// "지금 유량이 정상상태" 라는 가정으로 시작하면 지평 안에서 가장 큰 항을 틀리게 잡는다.
// 명령이 2 ms 마다 바뀌므로 실제 q 는 상당히 지연돼 있다. 그래서 매 틱 실제 인가 명령과
// 측정 압력으로 이 선형 필터를 함께 돌려 초기 상태를 갖고 간다 (안정한 2차계라 발산하지
// 않고, 오차원은 모델 불일치뿐이라 유계다).
//
// z 는 갱신하지 않는다 — AcadosMpc 가 compute_input_reference 에서 이미 갱신하므로
// 여기서 또 밀면 히스테리시스가 틱마다 두 번 전진한다. 호출자가 z/dir 을 채워 넣는다.
void  advance_valve_estimate(const PlantParams& p, ChannelState& s,
                             const std::array<float, 3>& u_applied,
                             const Exogenous& ex, float dt);

// ── MPPI 하이퍼파라미터 ────────────────────────────────────────────────────
struct Params {
  int   K{128};                // 샘플 수
  int   NP{10};                // 지평 스텝
  float Ts{0.004f};            // 스텝 길이 [s]
  int   substeps{1};           // 스텝당 오일러 서브스텝 (밸브 wn 대비 안정성)

  // λ: **비용 산포에 대한 비율**로 쓴다 (adaptive). λ_eff = λ·(Jmax − Jmin).
  // 절대 λ 는 비용 스케일이 바뀌면 매번 재튜닝해야 해서 쓰지 않는다.
  float lambda{0.30f};   // λ_eff = λ·(median(J) − min(J))  — 중앙값이라 꼬리에 견딘다
  // ── 탐색 분산: 크래킹 임계를 넘을 수 있어야 한다 ─────────────────────────
  // 이 밸브는 ≈52%(상류압 의존) 아래에서 유량이 정확히 0 이다. 정상상태에서 uref≈0 인데
  // 표준편차가 작으면 **모든 샘플이 임계 아래**라 궤적이 전부 같고 비용도 같아진다
  // (계측: median−min = 0.146 vs Jmin 2.27, 유효샘플 85%). 그러면 선택이 사라져
  // 명목이 무작위 보행으로 박스 끝에 박히고, 거기서 uref 가 커지는 순간 밸브를 세게 연다.
  //
  // 그래서 **혼합 분포**를 쓴다: 대부분은 작은 sigma 로 동작점을 정밀하게 다듬고,
  // explore_frac 만큼은 큰 sigma 로 임계를 건너뛴다. 데드밴드가 있는 플랜트에서
  // 단일 분산으로는 정밀도와 도달성을 동시에 못 얻는다.
  float sigma_pct{8.0f};           // 정밀 탐색 표준편차 [명령 %]
  float sigma_explore_pct{30.0f};  // 임계 돌파용 표준편차 [명령 %]
  float explore_frac{0.30f};       // 큰 sigma 를 쓰는 샘플 비율
  // 노이즈 1차 상관. 독립 노이즈는 250 Hz 성분이라 τ≈25 ms 밸브가 통과시키지 못해
  // 탐색이 낭비된다 → ε_k = β·ε_{k−1} + √(1−β²)·n_k 로 저역 성분에 힘을 준다.
  float noise_beta{0.70f};

  float w_track{1.0f};         // 추종 (정규화 오차²)
  float w_effort{0.02f};       // uref 로부터의 이탈²
  float w_du{0.05f};           // 스텝 간 변화율² (채터 억제)
  float track_scale_kpa{10.0f};// 오차 정규화 기준 [kPa]
  float terminal_mult{5.0f};   // 말단 상태 가중 배수

  float du_min{-100.f}, du_max{+100.f};  // uref 대비 보정 한계 [%]
  bool  taper_in_rollout{true};         // 롤아웃도 명령 테이퍼를 통과시킨다

  // 비용이 평평할 때(모든 샘플이 같은 궤적 → median−min ≈ 0) 갱신을 하면 Σŵε 가
  // 순수 잡음이 되어 명목이 무작위 보행한다. 그런 틱에는 갱신 대신 명목을 0(=순수
  // 피드포워드)으로 감쇠시킨다 — 정보가 없을 때의 안전한 기본값이다.
  float flat_spread{1.0e-3f};      // 이 값 이하의 (median−min) 은 "평평"으로 본다
  float flat_decay{0.90f};         // 평평한 틱의 명목 감쇠율
};

struct Stats {
  uint64_t calls{0};
  double   sum_us{0.0};        // 롤아웃 소요 합 [µs]
  float    max_us{0.f};
  double   sum_eff{0.0};       // 유효 샘플 수 Σ(1/Σŵ²) — 가중 붕괴 감지
  double   sum_cost{0.0};      // 채택 해의 비용
  double   sum_spread{0.0};    // 평균 초과비용 (mean − min)
  double   sum_outlier{0.0};   // (max − min)/(mean − min) — 이상치 지표
  uint64_t sat_first{0};       // 첫 스텝 보정이 박스에 붙은 횟수
  uint64_t flat{0};            // 비용이 평평해 갱신을 건너뛴 틱 수
};

// ── 솔버 ───────────────────────────────────────────────────────────────────
// 스레드당 1 인스턴스(채널당 1개)를 전제로 한다 — 내부 버퍼·RNG 를 공유하지 않는다.
// 핫패스에서 힙 할당이 없도록 생성 시 전부 예약한다.
class Solver {
public:
  Solver(const PlantParams& pp, const Params& pr, uint32_t seed);

  // x0    : 현재 추정 상태 (측정 압력 + 밸브 내부 상태 추정)
  // ex    : 외생 입력
  // uref  : 이번 틱의 피드포워드 명령 [%] 3개 (적분항 포함)
  // 반환  : 첫 스텝 보정 Δu [%] 3개. 호출자는 QP 결과와 동일하게 쓰면 된다.
  std::array<float, 3> solve(const ChannelState& x0, const Exogenous& ex,
                             const std::array<float, 3>& uref);

  void reset();                       // 명목 시퀀스 0 으로
  Stats take_stats();                 // 읽고 리셋 (주기 진단용)
  const Params& params() const { return pr_; }

private:
  // xoshiro128++ + Box-Muller. std::mt19937 + normal_distribution 은 이 호출량
  // (채널당 K·NP·3 = 3840/틱, 500 Hz)에서 무시할 수 없다.
  struct Rng {
    uint32_t s[4];
    bool  has_cache{false};
    float cache{0.f};
    explicit Rng(uint32_t seed);
    uint32_t next_u32();
    float uniform01();                // (0,1) 개구간
    float normal();
  };

  float rollout_cost(const ChannelState& x0, const Exogenous& ex,
                     const std::array<float, 3>& uref, int sample);

  PlantParams pp_;
  Params      pr_;
  Rng         rng_;
  Stats       st_{};

  int nseq_{0};                        // NP*3
  std::vector<float> nom_;             // 명목 Δu 시퀀스 [nseq]
  std::vector<float> noise_;           // 샘플별 원 노이즈 ε [K*nseq]
  std::vector<float> dnom_;           // 가중 평균 노이즈 [nseq]
  std::vector<float> cost_;            // 샘플별 비용 [K]
  std::vector<float> w_;               // 가중치 [K]
  std::vector<float> eps_;             // 한 샘플의 상관 노이즈 [nseq]
  std::vector<float> sortbuf_;        // 중앙값 산출용 [K]
};

}  // namespace mppi
