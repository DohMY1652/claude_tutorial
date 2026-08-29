#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <Eigen/Dense>

#include "PressureRefGen.hpp"

#include <deque>
#include <set>
#include <vector>
#include <array>
#include <mutex>
#include <thread>
#include <queue>
#include <condition_variable>
#include <functional>
#include <memory>
#include <atomic>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <string>

#include <qpOASES.hpp>
#include "Mppi.hpp"
#include "MppiSystem.hpp"

#ifdef __linux__
  #include <pthread.h>
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  #include <arpa/inet.h>
  #include <fcntl.h>
  #include <unistd.h>
  #include <errno.h>
#endif

// ================================
// Fixed sizes for this project
// ================================
static constexpr int NUM_CAN_BOARDS = 25;   // physical CAN boards (board_id 1..25)
static constexpr int PWM_PER_BOARD  = 3;    // v1, v2, v3 per board
static constexpr int PWM_TOTAL      = NUM_CAN_BOARDS * PWM_PER_BOARD;  // 51

static constexpr int PWM_CLAMP_MIN  = 0;
static constexpr int PWM_CLAMP_MAX  = 4095;

// Event-driven: board TX 500Hz → on_sensor 500Hz → all channels every tick
static constexpr int MPC_PHASES     = 1;   // 500Hz / 1 = 500Hz per channel (parallel via ThreadPool)

static constexpr int MPC_TOTAL      = 24;   // max channel capacity
static constexpr int MPC_OUT_DIM    = 3;

// QP Solver Wrapper (박스 제약 전용 — A_con 미사용)
class QP {
public:
    QP(int nv, int nc) : solver_(nv, nc) {
        options_.setToMPC();
        options_.printLevel = qpOASES::PL_NONE;
        solver_.setOptions(options_);
    }

    bool solve(const Eigen::MatrixXf& H, const Eigen::VectorXf& g,
               const Eigen::VectorXf& lb, const Eigen::VectorXf& ub,
               Eigen::VectorXf& solution)
    {
        Eigen::MatrixXd Hd = H.cast<double>();
        Eigen::VectorXd gd = g.cast<double>();
        Eigen::VectorXd lbd = lb.cast<double>();
        Eigen::VectorXd ubd = ub.cast<double>();

        auto extract = [&](int nrows) {
            Eigen::VectorXd sol(nrows);
            solver_.getPrimalSolution(sol.data());
            solution = sol.cast<float>();
        };

        ++n_calls_;

        // hot start 를 먼저 시도하고, 실패하면 **같은 틱에서** cold start 로 재시도한다.
        //
        // 예전에는 hot start 실패를 그대로 반환해 그 틱의 Δu 를 버렸다 (그 채널의 MPC 가
        // 그 틱만 피드포워드로 퇴화하고, 다음 틱에나 cold start 로 복구). 그런데 이 MPC 는
        // 매 틱 수치 야코비안으로 A·B 를 다시 만들고 u_ref 가 바뀌면 박스 경계(LL/UL)도
        // 전부 바뀌므로, active set 이 크게 달라져 hot start 가 실패하는 것은 **정상**이다.
        // 버릴 이유가 없고, 실패한 틱을 버리는 쪽이 오히려 제어를 간헐적으로 열어버린다.
        if (initialized_) {
            int nWSR = HOT_WSR;
            if (solver_.hotstart(Hd.data(), gd.data(), nullptr,
                                 lbd.data(), ubd.data(), nullptr, nullptr, nWSR)
                == qpOASES::SUCCESSFUL_RETURN) {
                extract((int)H.rows());
                return true;
            }
            initialized_ = false;
            ++n_hot_fail_;
        }

        int nWSR = COLD_WSR;
        initialized_ = (solver_.init(Hd.data(), gd.data(), nullptr,
                                     lbd.data(), ubd.data(), nullptr, nullptr, nWSR)
                        == qpOASES::SUCCESSFUL_RETURN);
        if (initialized_) { extract((int)H.rows()); return true; }

        ++n_hard_fail_;
        return false;
    }

    // 진단용 (건강하면 둘 다 0 에 가깝다). hot start 실패는 cold start 로 즉시 복구되어
    // 성능에 드러나지 않으므로 계측 없이는 보이지 않는다.
    struct Stats { int64_t calls, hot_fail, hard_fail; };
    Stats take_stats() {
        Stats s{n_calls_, n_hot_fail_, n_hard_fail_};
        n_calls_ = n_hot_fail_ = n_hard_fail_ = 0;
        return s;
    }

private:
    // hot start 는 직전 active set 을 재사용하므로 적은 반복으로 끝나야 정상이지만,
    // 매 틱 재선형화 때문에 10 회로는 자주 부족했다 (실측 실패율 참조). cold start 는
    // 처음부터 푸는 경로라 넉넉히 준다.
    static constexpr int HOT_WSR  = 30;
    static constexpr int COLD_WSR = 100;

    qpOASES::SQProblem solver_;
    qpOASES::Options options_;
    bool initialized_{false};
    int64_t n_calls_{0}, n_hot_fail_{0}, n_hard_fail_{0};
};

class ThreadPool {
public:
  explicit ThreadPool(size_t num_threads, const std::vector<int>& pin_cpus = {});
  ~ThreadPool();

  void enqueue(std::function<void()> fn);
  void run_batch_and_wait(std::vector<std::function<void()>>& tasks);

private:
  std::vector<std::thread> workers_;
  std::queue<std::function<void()>> queue_;
  std::mutex mtx_;
  std::condition_variable cv_;
  std::atomic<bool> stop_{false};
  std::vector<int> pin_cpus_;
};

// ============================================================================
// ControlAug — 실행 중 켜고 끌 수 있는 제어 보강
// ============================================================================
// 실기는 모델과 다르다. 어디가 다른지 미리 알 수 없으므로, **모르는 부분을 온라인으로
// 메우는 항**을 셋 두고 각각 독립적으로 켜고 끈다. 전부 기본 off 이므로 아무것도 켜지
// 않으면 기존 동작과 **비트 단위로 같다.**
//
// `ros2 param set /pack2/pp_controller aug.<이름> <값>` 으로 **재시작 없이** 바뀐다.
// 하나씩 켜 가며 효과를 분리해서 볼 수 있게 만든 것이 요점이다.
struct ControlAug {
  // ── ① 온라인 유량 이득 적응 ──────────────────────────────────────────
  // 밸브 모델의 절대 스케일(A_max)은 챔버 부피 추정에 통째로 비례한다. 부피는
  // 이중부피법으로 재도 ±30% 가 남고, 오리피스 면적비 환산도 스풀이 병목이면 틀린다.
  // 그래서 "모델이 예측한 유량 대비 실제 유량의 비" k_flow 를 매 틱 추정해
  // 역모델에 곱한다. k_flow=1 이면 모델 그대로다.
  //   측정 유량 q_meas = dP/dt·V/(R·T)   (챔버가 곧 유량계다)
  //   모델 유량 q_model = Σ_j q_static(적용된 명령)
  //   k ← k + rate·(q_meas/q_model − k)
  // 유량이 작을 때는 비가 잡음이라 갱신하지 않는다.
  bool  adapt_gain{false};
  float adapt_rate{0.20f};            // 창마다 적용하는 완화 계수
  float gain_min{0.25f}, gain_max{4.0f};
  float adapt_min_flow_lpm{0.10f};    // 이보다 작은 모델 유량에서는 갱신 안 함
  int   adapt_window{100};            // 최소자승 누적 창 [tick] (500 Hz 기준 0.2 s)

  // ── ② 오프셋 프리 (정상상태 외란 추정) ───────────────────────────────
  // 모델 오차·누설·센서 영점 이탈은 정상상태 오차로 남는다. 출력 외란 d 를 적분
  // 추정해 **레퍼런스를 그만큼 밀어** 실제 출력이 목표에 가게 한다 (offset-free MPC 의
  // 실용형). MPPI 비용에 손대지 않으므로 솔버 튜닝과 독립이다.
  //   d ← clamp(d + rate·(P_ref − P_meas))
  //   MPPI 가 추종하는 목표 = P_ref + d
  bool  offset_free{false};
  float dist_rate{0.5f};              // [1/s] — 제어 주기와 무관하게 초당 속도로 준다
  float dist_band_kpa{5.0f};          // 오차가 이 안일 때만 적분 (과도에서는 적분 금지)
  float dist_limit_kpa{30.0f};
  float dist_deadband_kpa{0.3f};      // 센서 분해능(0.25 kPa) 이하에서는 적분하지 않는다

  // ── ③ 접근 시상수 자동 조정 ──────────────────────────────────────────
  // mppi_ref_tau_s 가 성능을 가장 크게 좌우한다(MPPI.md 6.3). 오차 부호가 자주 바뀌면
  // (진동) 느리게, 한 방향으로 크게 남으면(둔함) 빠르게 민다. 경계 안에서만 움직인다.
  bool  auto_tune{false};
  float tune_rate{0.02f};             // 한 번에 바꾸는 비율
  float tau_min{0.06f}, tau_max{0.40f};
  float osc_hi{0.30f};                // 최근 창에서 부호 변화 비율이 이보다 크면 진동
  float err_slow_kpa{3.0f};           // 이보다 큰 오차가 한 방향으로 유지되면 둔하다
  int   tune_window{250};             // 판정 창 [tick] (500 Hz 기준 0.5 s)
};

class AcadosMpc {
public:
  struct Config {
    int can_board_id{4};   // physical CAN board ID (1-based); sensor = filt_out_[can_board_id-1]
    int global_id{0};
    int   NP{10};
    int   n_x{1};
    int   n_u{3};
    float Ts{0.004f};
    float Q_value{1.0f};
    float R_value{1.0f};
    float A_lin{1.0f};
    std::array<float,3> B_lin{1.0f, 0.0f, 0.0f};
    bool  is_positive{true};
    float pos_ki_micro{0.0f}, pos_ki_macro{0.0f}, pos_ki_atm{0.0f};
    float neg_ki_micro{0.0f}, neg_ki_macro{0.0f}, neg_ki_atm{0.0f};
    float ref_value{0.0f};
    float du_min{-100.0f};
    float du_max{+100.0f};
    float u_abs_min{0.0f};
    float u_abs_max{100.0f};
    float volume_m3{1.0e-5f};
    float prev_vol_m3{1.0e-5f};

    float last_ref_value_ = 101.325f;
    float ejector_k = 0.005f;
    float ejector_p_limit = 11.325f;
    float leakage_u_pos = 0.0f;
    float leakage_u_neg = 0.0f;
    float target_time_constant = 0.2f;
    // macro 를 여는 판정에 쓰는 micro 포화 기준 [%]. 100 = 레일 밸브를 완전히 열었는데도
    // 요구 유량을 못 낼 때만 macro 를 연다 (임의 임계값이 아니라 밸브 물리 한계).
    // 명령 테이퍼 폭 [kPa]. 오차가 이 안으로 들어오면 **크래킹 임계 위쪽 여유분**을
    // 연속적으로 줄인다. 하드 데드밴드를 대체한다 — 이유는 solve() 주석 참조.
    // "닫힘"으로 볼 유효면적 비율 (A_eff / A_max). 이 면적에 해당하는 전류가 크래킹
    // 임계이고, 그 이하에서는 솔레노이드 자기력이 스풀을 못 들어 유량이 0 이다.
    float valve_crack_area_frac = 1e-6f;
    // ── 명령 저역통과 [Hz] (0 = 끔) ────────────────────────────────────
    // 밸브는 2차계이고 실측 피팅이 ωn≈39.5~45.3 rad/s(6.3~7.2 Hz), ζ≈0.2 를 준다.
    // ζ=0.2 는 공진 첨두가 1/(2ζ)=2.5배인 **매우 약한 감쇠**다. 그런데 MPPI 는
    // du_limit=100 이라 한 틱(2 ms)에 u 를 0↔100 로 던질 수 있어 250 Hz 성분까지
    // 실린 명령이 나간다 — 그 스펙트럼이 7 Hz 공진을 정면으로 때린다.
    // 실기 계측: 챔버가 6.86 Hz 로 peak-to-peak 210 kPa 진동했고, 그 주파수가
    // 밸브 고유주파수와 정확히 일치했다. 첫스텝 포화도 97.6~100% 였다.
    // → 명령을 공진보다 한참 아래에서 잘라 낸다. 액추에이터가 따라올 수 없는
    //   빠르기로 명령하지 않는다는, 제어에서 가장 기본적인 규칙이다.
    float cmd_lpf_hz = 0.0f;
    float ki_u_limit_pct = 10.0f;  // 지령 트림 상한 [%p] — 크래킹 위치 오차용
    float ki_flow = 0.02f;         // 유량 트림 이득 [1/(kPa·s)]
    float q_trim_limit = 2.0f;     // 유량 배율 보정 상한 (2.0 = 최대 3배/1/3배)
    float crack_floor_rate_kpas = 5.0f;  // |dP/dt| 가 이보다 작을 때만 크래킹 하한을 건다
    float crack_floor_min_err_kpa = 1.5f; // 오차가 이보다 클 때만 크래킹 하한을 건다
    float integ_hold_rate_kpas = 0.0f;    // >0 이면: 압력이 그보다 빠르면 적분 멈춤 (0=끔)
    float integ_deadzone_boost = 1.0f;    // 데드존(무반응)에서 적분 배속 (1=평소대로)
    ControlAug aug{};                 // Controller 가 매 틱 최신값을 밀어 넣는다
    // 밸브별 13-parameter (0=micro, 1=macro, 2=atm). build_mpcs 가 채운다.
    // **이것이 모델의 단일 출처다.** 아래 평면 필드는 하위 호환용으로 micro 값을 담는다.
    std::array<mppi::PlantParams, 3> pv{};
    // 13-variable proportional valve model parameters
    float I_MAX{0.30f};
    float A_max{0.2845f};
    float k_shape{33.09f};
    float C_k{0.0288f};
    float C_p{0.00012f};
    float C_z{0.0f};
    float A_bw{260649.5f};
    float beta_bw{179.0f};
    float gamma_bw{0.06f};
    float alpha_shape{3884.2f};
    float wn_up{40.0f};
    float zeta_up{1.2f};
    float wn_down{45.0f};
    float zeta_down{1.0f};

    // ── 솔버 선택 ─────────────────────────────────────────────────────────
    // false: 기존 경로 (선형화 → 응축 QP → qpOASES)
    // true : MPPI (선형화 없음, 비선형 롤아웃 샘플링).  Mppi.hpp 머리말 참조.
    // 하네스가 비결정론적이라(README 0절) 두 경로를 남겨 A/B 비교할 수 있게 했다.
    bool  use_mppi{false};
    int   mppi_samples{128};
    float mppi_lambda{0.30f};
    float mppi_sigma_pct{8.0f};
    float mppi_sigma_explore_pct{30.0f};
    float mppi_explore_frac{0.30f};
    float mppi_du_limit_pct{100.0f};   // MPPI 경로의 Δu 한계 (QP 경로의 du_min/max 와 별개)
    // 지평 안 스테이지 레퍼런스의 접근 시상수 [s]. ≤0 이면 target_time_constant 를 쓴다.
    // 피드포워드보다 **빠른** 궤적을 주면 MPPI 가 그만큼 더 밀어붙인다.
    float mppi_ref_tau_s{-1.0f};
    // 계획 지평은 제어 주기와 **독립**이다. ≤0 이면 NP / Ts 를 그대로 쓴다.
    // 지평 길이 = mppi_np · mppi_ts_s. 밸브 τ≈25 ms 보다 충분히 길어야 밸브가
    // 응답하는 것을 지평 안에서 볼 수 있다.
    int   mppi_np{-1};
    float mppi_ts_s{-1.0f};
    float mppi_noise_beta{0.70f};
    // 음수면 Q_value / R_value 를 그대로 쓴다 (기존 튜닝 의미를 잇는다).
    float mppi_w_track{-1.0f};
    float mppi_w_effort{-1.0f};
    float mppi_w_du{0.05f};
    float mppi_track_scale_kpa{10.0f};
    float mppi_terminal_mult{5.0f};
    int   mppi_substeps{2};
    // 롤아웃 초기 상태로 필터 전 생값을 쓴다. 측정 지연 가설의 싼 **진단** 스위치.
    // 계측(스텝 4개 평균): 압력 RMSE 5.51 → 5.09, 정상상태 압력오차 0.25 → 0.00 으로
    // 개선되지만 위치 IAE 는 5.30 → 6.43 으로 악화된다. 생값은 양자화 잡음(양압 보드
    // 0.25 kPa/LSB)이 그대로라 MPPI 가 잡음을 쫓아 밸브가 떨린다. 즉 지연 가설은 맞지만
    // 해법은 생값이 아니라 **관측기**다 (아래).
    bool  mppi_raw_state{false};

    // ── 상태 관측기 ────────────────────────────────────────────────────────
    // 필터값은 매끄럽지만 ≈18 ms 늦고(LPF 직렬 2단, 각 τ≈9 ms — 지평 40 ms 의 45%),
    // 생값은 즉각이지만 시끄럽다. 모델로 앞서 예측하고 그 예측에 **같은 LPF 2단을
    // 복제**해 측정과 같은 조건으로 비교한 뒤 잔차로 보정하면 둘을 동시에 얻는다.
    // MPPI 에는 필터가 안 걸린 추정값을 넘긴다.
    bool  mppi_estimator{false};
    float obs_gain{0.10f};          // 잔차 보정 이득 [1/tick]
    float obs_bridge_alpha{0.2f};   // 브리지/시뮬 LPF 계수 (컨트롤러가 직접 모르는 값)
    float obs_ctrl_alpha{0.2f};     // 컨트롤러 LPF 계수 (sensor_filter_alpha 를 그대로 받는다)
  };

  explicit AcadosMpc(const Config& cfg);
  void set_qp_solver(std::shared_ptr<QP> qp);
  inline void set_ref_value(float ref_kpa) { cfg_.ref_value = ref_kpa; }
  inline void set_volume(float vol_m3) { cfg_.volume_m3 = std::max(1e-12f, vol_m3); }
  inline void set_prev_volume(float vol_m3) { cfg_.prev_vol_m3 = std::max(1e-12f, vol_m3); }
  void update_linearization(float x_ref, const Eigen::RowVector3f& u_ref);
  void set_AB_sequences(const std::vector<float>& A_seq, const std::vector<Eigen::RowVector3f>& B_seq);
  void set_AB_constant(float A_scalar, const Eigen::RowVector3f& B_row);
  void solve(float dt_ms, std::array<uint16_t, MPC_OUT_DIM>& out3, float current_time_sec);
  // 중앙집중 MPPI 용 분리 — 그 사이에 전체 시스템 솔버가 끼어든다.
  // prepare: z 갱신 · 피드포워드 · 적분항 · 크래킹 임계 → uref
  // finish : 테이퍼 · 클램프 · PWM · 밸브 상태 추정 전진
  std::array<float,3> prepare(float dt_ms, float current_time_sec);
  void finish(const std::array<float,3>& du3, std::array<uint16_t, MPC_OUT_DIM>& out3);
  void report_mppi_stats();
  inline float p_used() const { return P_used_; }          // 실제 쓴 압력 (추정/생/필터)
  inline const std::array<float,3>& u_crack() const { return u_crack_; }
  inline const std::array<float,3>& uref()    const { return uref_; }
  inline const mppi::ChannelState& plant_est() const { return plant_est_; }
  inline float vol_dot_est() const { return vol_dot_est_; }
  inline float k_flow()   const { return k_flow_; }
  inline float d_hat()    const { return d_hat_; }
  inline float tau_used() const { return tau_ref_cur_; }
  // 롤아웃 초기 상태 — 채널 경로와 중앙집중 경로가 **같은 조립 규칙**을 쓰게 한다.
  // prepare() 직후에 부를 것 (z 가 이번 틱 값으로 갱신된 뒤여야 한다).
  mppi::ChannelState rollout_state() const;
  const mppi::ChannelPlant& plant_params() const { return mppi_pv_; }
  const Config& cfg() const { return cfg_; }
  Config& cfg_mutable() { return cfg_; }   // 보강 설정을 매 틱 밀어 넣기 위해

  // macro 개방 게이트는 없다. 분담량이 곧 개방 여부다 —
  // compute_input_reference 의 split_demand 주석 참조.
  // 이 채널이 붙은 레일의 압력 변화율 [kPa/s]. 컨트롤러가 12채널 명목 명령으로 한 번
  // 계산해 공유한다 — 롤아웃이 레일을 상수로 두던 오차(≈5~34 kPa)를 없앤다.
  inline void set_rail_rate(float r) { rail_rate_ = r; }
  // 과압 세이프티가 래치되면 컨트롤러 명령이 **무시되고** 밸브가 강제 전개된다.
  // 그동안의 추종 오차는 컨트롤러 탓이 아니므로 적분(오프셋 프리)을 멈춰야 한다.
  inline void set_safety_latched(bool v) { safety_latched_ext_ = v; }

  float current_P_now_       = 101.325f;
  // 롤아웃 초기 상태 전용 생값(필터 전). 오차·피드포워드·적분항은 그대로 필터값을 쓴다 —
  // 그래야 "초기 상태만" 바꾼 효과를 분리해 볼 수 있다.
  float current_P_now_raw_   = 101.325f;
  float current_P_micro_     = 101.325f;
  float current_P_macro_     = 101.325f;
  float current_P_macro_neg_ = 101.325f;
  float current_P_atm_       = 101.325f;

private:
  // Bouc-Wen hysteresis state estimates for each valve (micro/atm/macro)
  double z_micro_{0.0}, prev_I_micro_{0.0};
  double z_atm_{0.0},   prev_I_atm_{0.0};
  double z_macro_{0.0}, prev_I_macro_{0.0};
  int    dir_micro_{0},  dir_atm_{0},  dir_macro_{0};
  // 밸브별 크래킹 임계 [%] — compute_input_reference 가 매 틱 현재 Pin/z 로 갱신한다.
  // 순서는 last_u3_ 와 동일: [0]=micro, [1]=macro, [2]=atm.
  std::array<float,3> u_crack_{0.f, 0.f, 0.f};
  std::array<float,3> compute_input_reference(float P_now, float P_micro, float P_macro, float P_macro_neg, float dt_sec, float current_time_sec);
  void build_mpc_qp(const std::vector<float>& A_seq, const std::vector<Eigen::RowVector3f>& B_seq, float P_now, const std::vector<float>& P_ref, Eigen::MatrixXf& P, Eigen::VectorXf& q, Eigen::MatrixXf& A_con, Eigen::VectorXf& LL, Eigen::VectorXf& UL);
  std::array<float,3> solve_qp_first_step(const Eigen::MatrixXf& P, const Eigen::VectorXf& q, const Eigen::MatrixXf& A_con, const Eigen::VectorXf& LL, const Eigen::VectorXf& UL);

private:
  Config cfg_;
  std::shared_ptr<QP> qp_;
  std::vector<float> P_ref_;
  std::vector<float> A_seq_;
  std::vector<Eigen::RowVector3f> B_seq_;
  Eigen::MatrixXf Q_, R_, Pmat_, Acon_;
  Eigen::VectorXf qvec_, LL_, UL_;
  std::array<float,3> last_u3_{0,0,0};
  float pos_error_integral_{0.0f};
  // ── 보강 상태 (채널별) ──────────────────────────────────────────────
  float k_flow_{1.0f};        // ① 적응된 유량 이득 (1 = 모델 그대로)
  float d_hat_{0.0f};         // ② 추정 출력 외란 [kPa]
  float tau_ref_cur_{-1.0f};  // ③ 현재 접근 시상수 [s] (<0 = 아직 초기화 전)
  int   sign_flips_{0}, tune_tick_{0};
  int   last_err_sign_{0};
  float err_abs_acc_{0.0f};
  float q_model_last_{0.0f};  // 직전 틱에 적용한 명령의 모델 유량 [LPM]
  double adapt_sxy_{0.0}, adapt_sxx_{0.0};   // Σ(q_meas·q_model), Σ(q_model²)
  int    adapt_n_{0};
  float ref_eff_{101.325f};   // 외란 보정이 들어간 유효 레퍼런스 [kPa]
  // 적분 보정 [지령 %p]. uref 가 아니라 **MPPI 뒤**에 더한다 — 자세한 이유는
  // AcadosMpc::finish() 의 주석 참조.
  float ki_flow_{0.02f};      // 유량 트림 이득
  float q_trim_{1.0f};        // 이번 틱의 유량 배율 (진단용)
  float err_abs_{0.0f};       // 이번 틱의 |Pref − P| [kPa]
  float want_sign_{0.0f};     // +1 = 압력을 올리려는 요구, −1 = 내리려는 요구
  std::array<bool,3>  u_want_{false, false, false};  // 이 틱에 유량을 요구한 밸브
  std::array<float,3> u_trim_{0.f, 0.f, 0.f};
  std::array<float,3> u_lpf_{0.f, 0.f, 0.f};   // 명령 저역통과 상태
  bool  u_lpf_init_{false};
  bool  safety_latched_ext_{false};
  float dpdt_f_{0.0f};        // 측정 압력 변화율 [kPa/s], τ=100 ms
  float p_prev_meas_{-1.0f};  // 직전 틱 측정압 [kPa] (유량 역산용)
  float neg_error_integral_{0.0f};

  float last_error_{0.0f};

  std::deque<float> vol_dot_buffer_;
  const size_t vol_dot_window_size_ = 5;

  int qp_fail_count_{0};
  int nonfinite_cnt_{0};           // 비유한 명령 차단 횟수 (실기 안전 진단)
  int qp_stat_tick_{0};

  // ── MPPI 경로 ────────────────────────────────────────────────────────────
  // plant_est_ 는 밸브 2차 동특성 상태(q, qd) 추정이다. 롤아웃 초기값으로 쓰고,
  // 매 틱 실제 인가 명령 + 측정 압력으로 함께 전진시킨다. z 는 여기 두지 않고
  // z_micro_/z_atm_/z_macro_ 를 단일 출처로 삼아 매 틱 복사해 넣는다.
  std::unique_ptr<mppi::Solver> mppi_;
  mppi::ChannelPlant            mppi_pv_{};
  mppi::ChannelState            plant_est_{};
  int   mppi_stat_tick_{0};
  float vol_dot_est_{0.0f};        // compute_input_reference 가 매 틱 갱신 [m³/s]
  // prepare → finish 사이에 넘겨야 하는 값들
  std::array<float,3> uref_{0.f,0.f,0.f};
  float dt_sec_{0.004f};
  float P_used_{101.325f};         // 이번 틱에 실제로 쓴 압력 (추정/생/필터 중 하나)
  float rail_rate_{0.0f};          // 레일 압력 변화율 [kPa/s] (컨트롤러가 주입)

  // ── 관측기 상태 ──────────────────────────────────────────────────────────
  // p_hat_      : 필터가 걸리지 않은 **진짜** 챔버압 추정 (MPPI 초기 상태로 쓴다)
  // p_hat_f1/f2 : 그 추정에 브리지 LPF·컨트롤러 LPF 를 복제 적용한 값.
  //               측정값(filt_out_)과 같은 지연을 가지므로 직접 비교할 수 있다.
  float p_hat_{101.325f}, p_hat_f1_{101.325f}, p_hat_f2_{101.325f};
  bool  p_hat_init_{false};
  double obs_resid_acc_{0.0};      // 진단: 잔차 절대값 누적
  int    obs_resid_n_{0};

  Eigen::MatrixXf S_bar_, T_bar_;
  Eigen::VectorXf x0_mpc_, Xref_mpc_, qtmp_, solution_;
};

struct SensorCalib {
  struct Channel { double offset{1.0}; double gain{250.0}; };
  double atm_offset{101.325};
  std::array<Channel, NUM_CAN_BOARDS> boards{};   // boards[0] = board_id 1
  double kpa_atm() const { return atm_offset; }
  double kpa(int board_id, uint16_t raw) const {
    int idx = board_id - 1;
    if (idx < 0 || idx >= (int)boards.size()) return kpa_atm();
    const auto& c = boards[(size_t)idx];
    return (double(raw) - c.offset) * c.gain + kpa_atm();
  }
};

class RefTcpClient {
public:
    struct Config {
        bool enable = false;
        std::string host = "169.254.46.254";
        int port = 2272;
        int expect_n = 12;
        double pressure_scale = 1.0 / 327.675;
    };
    using Callback = std::function<void(const std::vector<double>&)>;

    RefTcpClient(const Config& cfg, Callback cb)
    : cfg_(cfg), cb_(std::move(cb)) {
#ifdef __linux__
        th_ = std::thread([this](){ run_(); });
#else
        (void)cfg_; (void)cb_;
#endif
    }

    ~RefTcpClient() {
        stop_.store(true);
#ifdef __linux__
        if (client_fd_ >= 0) ::shutdown(client_fd_, SHUT_RDWR);
#endif
        if (th_.joinable()) th_.join();
    }

private:
    void run_() {
#ifndef __linux__
        return;
#else
        const size_t NUM_INTEGERS = (size_t)cfg_.expect_n;
        const size_t BYTES_PER_INT = sizeof(uint16_t);
        const size_t BUFFER_SIZE = NUM_INTEGERS * BYTES_PER_INT;

        std::vector<char> buffer(BUFFER_SIZE);
        std::vector<double> out_values(NUM_INTEGERS);

        while (!stop_.load()) {
            client_fd_ = -1;
            try {
                client_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
                if (client_fd_ < 0) throw std::runtime_error("Socket creation failed");

                sockaddr_in server_addr{};
                server_addr.sin_family = AF_INET;
                server_addr.sin_port = htons((uint16_t)cfg_.port);

                if (::inet_pton(AF_INET, cfg_.host.c_str(), &server_addr.sin_addr) <= 0)
                    throw std::runtime_error("Invalid address/ Address not supported");

                RCLCPP_INFO(rclcpp::get_logger("RefTcpClient"), "Connecting to reference server %s:%d...", cfg_.host.c_str(), cfg_.port);

                if (::connect(client_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
                    throw std::runtime_error("Connection Failed");

                RCLCPP_INFO(rclcpp::get_logger("RefTcpClient"), "Reference server connected.");

                while (!stop_.load()) {
                    size_t total_recd = 0;
                    while (total_recd < BUFFER_SIZE && !stop_.load()) {
                        ssize_t n = ::recv(client_fd_, buffer.data() + total_recd, BUFFER_SIZE - total_recd, 0);
                        if (n == 0) throw std::runtime_error("Server disconnected");
                        if (n < 0) {
                            if (errno == EINTR) continue;
                            throw std::runtime_error(std::string("Recv error: ") + strerror(errno));
                        }
                        total_recd += (size_t)n;
                    }

                    if (total_recd == BUFFER_SIZE) {
                        const char* ptr = buffer.data();
                        for (size_t i = 0; i < NUM_INTEGERS; ++i) {
                            uint16_t net_val;
                            std::memcpy(&net_val, ptr, BYTES_PER_INT);
                            ptr += BYTES_PER_INT;
                            uint16_t host_val = ntohs(net_val);
                            out_values[i] = static_cast<double>(host_val) * cfg_.pressure_scale;
                        }
                        cb_(out_values);
                    }
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("RefTcpClient"), "%s", e.what());
                if (client_fd_ >= 0) { ::close(client_fd_); client_fd_ = -1; }
                if (!stop_.load()) {
                    RCLCPP_INFO(rclcpp::get_logger("RefTcpClient"), "Reconnecting in 5 seconds...");
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                }
            }
        }
        if (client_fd_ >= 0) ::close(client_fd_);
#endif
    }

    Config cfg_;
    Callback cb_;
    std::thread th_;
    std::atomic<bool> stop_{false};
    int client_fd_ = -1;
};

class RefTcpServer {
public:
    struct Config {
        bool enable = false;
        int port = 2293;
        int pos_gid = 0;
        int neg_gid = 6;
        int num_values = 2;   // doubles per TCP message (2 = pressure mode, N = N-axis angle mode)
    };
    using Callback = std::function<void(const std::vector<double>&)>;

    RefTcpServer(const Config& cfg, Callback cb);
    ~RefTcpServer();

private:
    void run_();
    Config cfg_;
    Callback cb_;
    std::thread th_;
    std::atomic<bool> stop_{false};
    std::atomic<int> server_fd_{-1};
    std::atomic<int> client_fd_{-1};
};

class Controller : public rclcpp::Node {
public:
  explicit Controller(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());
  ~Controller() override;

private:
  void on_sensor(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
  void on_volume(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void on_timer();
  void build_mpcs();
  void on_zero_calibration(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res);
  void inner_loop_1khz(float dt_ms);
  inline uint16_t clamp_pwm(int v) const { return static_cast<uint16_t>( std::min(std::max(v, PWM_CLAMP_MIN), PWM_CLAMP_MAX) ); }
  void publish_cmds();

  // 위치 제어: 엔코더 각도 → 압력 레퍼런스 변환 (on_timer 내 호출)
  void run_position_control(double dt_sec);

  // control_mode 2: 위치 PID → 목표 토크 → 최적화 생성기 → 12개 목표 압력
  void run_optimized_pressure_ref(double dt_sec);

private:
  double sensor_filter_alpha_{1.0};
  std::vector<double> filt_state_;                      // [NUM_CAN_BOARDS]
  std::array<double,   NUM_CAN_BOARDS> filt_out_{};     // kPa, indexed by board_id-1
  // 필터 **전** 압력. 측정 경로에 LPF 가 직렬 2단(브리지 α=0.2 + 여기 α=0.2, 각 τ≈9 ms)
  // 걸려 있어 filt_out_ 은 약 18 ms 낡았고, 그것은 MPPI 지평 40 ms 의 45% 다.
  // 예측기의 초기 상태로는 생값이 더 나을 수 있어 비교용으로 함께 들고 간다.
  std::array<double,   NUM_CAN_BOARDS> raw_out_{};
  bool filter_initialized_{false};

  std::vector<double> ref_snapshot_;
  std::vector<double> final_active_vols_ml_;

  int period_ms_{2};
  bool enable_thread_pinning_{true};
  std::vector<int64_t> cpu_pins_param_;

  int num_positive_channels_{8};
  int num_total_channels_{12};
  int num_actuators_{1};   // 액추에이터(축) 수 → 압력채널/엔코더/위치제어기 모두 이 값만큼 활성화

  int channel_board_offset_{4};   // board_id = gid + channel_board_offset
  int P_pos_board_id_{1};         // board carrying P_line_pos sensor
  int P_neg_board_id_{2};         // board carrying P_line_neg sensor
  int P_macro_board_id_{3};       // board carrying P_line_macro sensor
  int P_macro_neg_board_id_{4};   // board carrying P_line_macro_neg sensor

  // 13-variable 비례밸브 모델 파라미터 한 세트.
  //
  // **밸브마다 다르다.** `RUNBOOK.md` 의 밸브 피팅은 채널당 3개(micro/atm/macro)를 따로
  // 맞추고 `valve_fit_solve.py` 가 `channel_config.chN.{micro,atm,macro}.*` 로 쓴다.
  // 예전에는 로더가 채널당 한 세트만 읽어 세 밸브에 같은 값을 써서 **피팅 결과를 쓸 수
  // 없었다** (README 8.8 의 "밸브별 파라미터 로더 미완"). 이제 세 세트를 읽고, 없으면
  // 평면 `chN.*` 로 폴백한다.
  struct Valve13 {
    double I_MAX{0.30};
    double A_max{0.2845};
    double k_shape{33.09};
    double C_k{0.0288};
    double C_p{0.00012};
    double C_z{0.0};
    double A_bw{260649.5};
    double beta_bw{179.0};
    double gamma_bw{0.06};
    double alpha_shape{3884.2};
    double wn_up{40.0};
    double zeta_up{1.2};
    double wn_down{45.0};
    double zeta_down{1.0};
  };

  struct ChannelConfig {
    double pos_ki_micro{0.0};
    double pos_ki_macro{0.0};
    double pos_ki_atm{0.0};
    double neg_ki_micro{0.0};
    double neg_ki_macro{0.0};
    double neg_ki_atm{0.0};
    // 밸브별 13-parameter. 인덱스는 mppi::ValveIdx 와 동일 (0=micro, 1=macro, 2=atm).
    std::array<Valve13, 3> v{};
    double chamber_volume_ml{-1.0};   // 피팅으로 구한 챔버 부피 (<0 이면 미측정)
    bool   per_valve_loaded{false};   // 진단용 — 피팅 파일이 실제로 로드됐는지
  };

  std::vector<ChannelConfig> channel_configs_;

  double default_volume_ml_{1.0};
  bool   actuator_connected_{true};
  double tank_volume_pos_ml_{750.0};
  double tank_volume_neg_ml_{400.0};

  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_sensors_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_volumes_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_analog_;
  std::array<double, 9> encoder_angles_{};   // boards 17..25 [deg], index 0 = board 17
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_pwm_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_mpc_refs_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_active_vols_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_kpa_all_;

  std::unique_ptr<ThreadPool> pool_;
  size_t pool_threads_{2};
  std::vector<std::function<void()>> sys_tasks_;   // 핫패스 재할당 방지

  // ── 중앙집중 MPPI (solver: mppi_system) ──────────────────────────────────
  // 12개 채널 + 라인 밸브 2개를 **하나의 최적화**로 푼다. 레일 강하와 채널 유량 요구
  // 총합이 같은 모델 안에 들어오므로, 채널별 독립 MPPI 가 레일을 무한 소스로 가정해
  // 생기던 ≈34 kPa(6채널 동시) 예측 오차가 원리적으로 사라진다. 라인 PID 는 흡수된다.
  std::unique_ptr<mppi::SystemSolver> sys_mppi_;
  mppi::SysParams    sys_params_{};
  mppi::SysState     sys_state_{};
  mppi::SysExo       sys_exo_{};
  std::vector<float> sys_uref_;
  bool  sys_init_{false};
  bool  sys_control_lines_{false};
  double sys_deadline_us_{1200.0};
  int    sys_over_budget_{0};
  long   sys_over_cnt_{0}, sys_skipped_{0};
  int   sys_stat_tick_{0};
  double sys_pred_err_pos_{0.0}, sys_pred_err_neg_{0.0};
  double sys_pred1_pos_{0.0}, sys_pred1_neg_{0.0};
  bool   sys_pred1_valid_{false};
  int    sys_pred_n_{0};
  void build_system_mppi();
  void estimate_rail_rates(double P_line_pos_kPa, double P_line_neg_kPa,
                           double P_atm_kPa, float& dPpos_dt, float& dPneg_dt);
  float rail_rate_pos_{0.0f}, rail_rate_neg_{0.0f};
  bool  rail_rate_enable_{false};
  void run_system_mppi(double P_atm_kPa, double P_line_pos_kPa, double P_line_neg_kPa,
                       double P_line_macro_kPa, double P_line_macro_neg_kPa);
  std::mutex sensors_mtx_;
  std::array<uint16_t, NUM_CAN_BOARDS> sensors_raw_{};   // indexed by board_id-1

  std::array<uint16_t, PWM_TOTAL> zoh_{};    // [(board_id-1)*3 + v_idx]
  std::array<int,      PWM_TOTAL> inner_{};
  std::array<uint16_t, PWM_TOTAL> cmds_{};

  std::vector<std::unique_ptr<AcadosMpc>> mpcs_;
  uint64_t tick_{0};
  double wall_elapsed_sec_{0.0};   // 실제 벽시계 경과 — 틱 간격 진단용 (제어에는 쓰지 않는다)
  SensorCalib sensor_;

  struct MpcYaml {
    int   NP{5}; int n_x{1}; int n_u{3}; double Ts{0.01}; double Q_value{10.0}; double R_value{1.0};
    double ejector_k{0.005};
    double ejector_p_limit{11.325};
    double leakage_u_pos{0.0};
    double leakage_u_neg{0.0};
    double target_tc{0.2};
    double valve_crack_area_frac{1e-6};
    double cmd_lpf_hz{0.0};   // 명령 저역통과 [Hz], 0=끔
    double ki_u_limit_pct{10.0};  // 지령 트림 상한 [%p]
    double ki_flow{0.02};
    double q_trim_limit{2.0};
    double crack_floor_rate_kpas{5.0};
    double crack_floor_min_err_kpa{1.5};
    double integ_hold_rate_kpas{0.0};
    double integ_deadzone_boost{1.0};
    // 솔버 선택 + MPPI 하이퍼파라미터
    std::string solver{"qp"};
    int    mppi_samples{128};
    double mppi_lambda{0.30};
    double mppi_sigma_pct{8.0};
    double mppi_sigma_explore_pct{30.0};
    double mppi_explore_frac{0.30};
    double mppi_du_limit_pct{100.0};
    double mppi_ref_tau_s{-1.0};
    int    mppi_np{-1};
    double mppi_ts_s{-1.0};
    double mppi_noise_beta{0.70};
    double mppi_w_track{-1.0};
    double mppi_w_effort{-1.0};
    double mppi_w_du{0.05};
    double mppi_track_scale_kpa{10.0};
    double mppi_terminal_mult{5.0};
    int    mppi_substeps{2};
    bool   mppi_raw_state{false};
    bool   mppi_estimator{false};
    double obs_gain{0.10};
    double obs_bridge_alpha{0.2};
  } mpc_;

  std::vector<double> vol_ml_;

  bool sys_valve_operate_{false};
  RefTcpClient::Config ref_client_cfg_;
  std::unique_ptr<RefTcpClient> ref_client_;
  RefTcpServer::Config ref_server_cfg_;
  std::unique_ptr<RefTcpServer> ref_server_;
  std::mutex mpc_ref_mtx_;
  std::vector<double> mpc_ref_kpa_;

  struct PidGains { double kp{0.5}, ki{0.0}, kd{0.0}, ref{150.0}; };
  struct PidState { double integ{0.0}; double prev_err{0.0}; bool has_prev{false}; };
  PidGains pid_pos_;
  PidState pid_pos_state_;
  double pid_out_min_{0.0}, pid_out_max_{100.0};
  int    pid_pos_pwm_index_{0};    // flat index into zoh_: (board_id-1)*3 + v_idx
  PidGains pid_neg_;
  PidState pid_neg_state_;
  int    pid_neg_pwm_index_{3};    // flat index into zoh_

  int    macro_switch_pwm_index_{3};   // flat index into zoh_

  // ──────────────────────────────────────────
  // 위치 제어기
  // ──────────────────────────────────────────
  int control_mode_{0};   // 0: 압력 제어, 1: 위치 제어

  // 축(actuator) 하나의 위치 제어 설정.
  //
  // mode 1(휴리스틱)과 mode 2(최적화 생성기)가 같은 구조체를 공유하지만 쓰는 필드가
  // 다르다. 어느 값이 살아 있는지 yaml 만 보고 알 수 없었으므로 mode 1 전용 게인을
  // `m1` 하위 구조체로 분리했다. mode 2 는 `m1` 을 전혀 읽지 않는다 — PID 는
  // TorquePID.axis*, 압력 분배는 PressureRefGen 의 슬루 박스 최적화가 담당한다.
  struct PositionCtrlConfig {
    // ── 공용 (mode 1 / 2) ──────────────────────────────────────────────
    // 채널 / 엔코더 매핑
    int    actuator_idx{0};      // 엔코더 인덱스 (0 = board 17)
    int    pos_gid{0};
    int    neg_gid{6};
    // 중력 피드포워드용 부하 (mode 1 은 kPa 로 환산, mode 2 는 N·m 로 직접 사용)
    double mass_kg{1.0};
    double link_length_m{0.2};
    // 채널 압력 정격 [kPa abs] — mode 1 은 출력 클램프, mode 2 는 생성기 슬루 박스의
    // 상/하한으로 쓴다 (Controller 생성자에서 게이지 Pa 로 변환해 주입).
    double p_pos_max_kpa{165.0};
    double p_neg_min_kpa{70.0};
    // 각속도 추정 LPF / 초기 목표각
    double vel_filter_alpha{0.05};
    double default_angle_deg{0.0};

    // ── mode 1 (휴리스틱 위치 제어) 전용 ────────────────────────────────
    struct Mode1 {
      // PID 게인 (각도 오차 → 압력 보정)
      double kp{3.0};              // [kPa/deg]
      double ki{0.05};             // [kPa/(deg·s)]
      double kd{0.02};             // [kPa·s/deg]
      double integral_limit_kpa{20.0};
      // 중력 FF 환산 계수 [kPa/(N·m)] — mode 2 는 환산이 불필요해 쓰지 않는다
      double kff_gravity{10.0};
      // 쿨롱 마찰 보상
      double friction_kpa{2.0};
      // 바이어스(평형점) 압력 + 음압 차동 비율
      double p_bias_pos_kpa{120.0};
      double p_bias_neg_kpa{90.0};
      double neg_coupling{0.5};
      // 출력 제한 중 mode 1 만 쓰는 쪽 (반대쪽 한계는 공용 정격)
      double p_pos_min_kpa{101.325};
      double p_neg_max_kpa{101.325};
      // 압력 레퍼런스 슬루레이트 제한 [kPa/s]. 목표압력이 한 번에 점프하면 밸브모델-실제
      // 불일치로 안전한계에 부딪히는 릴레이 진동이 난다 (20260818).
      // mode 2 는 생성기의 슬루 박스가 같은 역할을 물리적으로 하므로 불필요하다.
      double ref_slew_kpa_per_s{3.0};
    } m1;
  };

  struct PositionCtrlState {
    double integral{0.0};
    double prev_angle{0.0};
    double vel_filt{0.0};    // 필터된 각속도 [deg/s]
    bool   initialized{false};
    double p_pos_ref_filt{101.325};   // 슬루레이트 제한 후 마지막 P+ 레퍼런스 [kPa]
    double p_neg_ref_filt{101.325};   // 슬루레이트 제한 후 마지막 P- 레퍼런스 [kPa]
  };

  // 축(actuator)별 위치 제어기 설정/상태. 크기 = num_actuators_
  std::vector<PositionCtrlConfig> pos_ctrl_cfg_;
  std::vector<PositionCtrlState>  pos_ctrl_state_;

  // TCP에서 수신한 축별 목표 각도 (mpc_ref_mtx_ 로 보호). 크기 = num_actuators_
  std::vector<double> target_angle_deg_;
  std::vector<double> target_angle_slewed_;   // 슬루 제한을 지난 목표 (제어가 쓰는 값)
  double target_slew_dps_{0.0};               // 각도 목표 슬루 [deg/s], 0=끔
  // 목표가 측정각보다 앞설 수 있는 최대 오차 [deg]. 0 이하면 끔.
  // 한 방향 힘 시스템이라 하강은 중력에 맡길 수밖에 없다 — 목표가
  // 달아나면 τ_ref 가 0 으로 떨어져 자유낙하한다 (S-30 참조).
  double target_follow_band_deg_{5.0};
  void slew_targets(double dt_sec);
  bool   pos_tcp_received_{false};

  // 위치 제어 디버그 토픽
  // data: 축마다 8개씩 이어붙임 [angle, angle_ref, p_pos_ref, p_neg_ref, p_pid, p_ff, p_friction, vel_dps] × num_actuators_
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pos_dbg_;

  // ──────────────────────────────────────────
  // control_mode 2: 최적화 기반 압력 레퍼런스 생성기
  // ──────────────────────────────────────────
  std::unique_ptr<PressureRefGen> refgen_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_refgen_dbg_;

  int      gen_period_ms_{20};       // 생성기 주기 (제어 tick 여러 개마다 1회)
  // 생성기가 낸 압력 레퍼런스의 슬루 제한 [kPa/s]. 0 이하면 끔.
  // 모드 1 의 ref_slew_kpa_per_s 와 같은 역할인데 모드 2 에는 없었다.
  double   gen_ref_slew_kpa_s_{150.0};
  uint64_t gen_tick_{0};
  bool     gen_use_ej_meas_{true};   // board 4 측정 음압을 이젝터 하류압으로 사용

  // 액추에이터 기하 (단일 출처). 부피식·토크 환산이 모두 여기서 나온다.
  double piston_area_mm2_{M_PI * 25.0 * 25.0};   // Ø50 mm
  double reel_radius_mm_{25.0};                  // 조인트 ~ 피스톤 로드 (= 부피식 mm/rad)
  double vol_offset_pos_mm_{40.0};
  double vol_offset_neg_mm_{90.0};

  // 축별 토크 PID (mode 2). 게인 단위는 N·m/deg 계열.
  struct TorquePid {
    double kp{0.0786}, ki{0.0295}, kd{0.0049};
    double integ_limit_nm{2.0};
    double friction_nm{0.30};
    // 마찰 보상 밴드 [deg]. 오차가 이 안이면 friction_nm 을 선형으로 준다.
    // 0 에 가까우면 예전의 하드 sign 과 같아져 목표 근처에서 ±friction_nm 이
    // 계단으로 뒤집힌다 (S-29 참조).
    double friction_band_deg{1.0};
    // 중력 피드포워드 배율. 액추에이터 미연결 시험에서 목표 압력을 낮추는 데 쓴다.
    // 목표 압력에 거의 선형으로 반영된다. 액추에이터를 붙이면 1.0 으로 되돌릴 것.
    double tau_ff_gain{1.0};
  };
  std::vector<TorquePid>  tau_pid_;
  std::vector<double>     tau_integ_;

  // macro 게이트 임계 — 생성기의 축별 **유량 부족률** [0,1] 이 이 값을 넘으면 macro 를 연다.
  // 무차원이라 "레일이 이번 스텝 수요의 몇 %를 못 대면 부스트를 부른다"로 읽힌다.
  std::vector<double> gen_starve_pos_, gen_starve_neg_;   // 진단용 [%]

  // gid → MPC 조회 (macro 게이트 설정용)
  AcadosMpc* mpc_for_gid(int gid) const;

  // 생성기 결과 ZOH (생성기 주기 사이 유지)
  std::vector<double> gen_pos_ref_kpa_, gen_neg_ref_kpa_;
  double gen_rail_pos_sp_kpa_{155.0}, gen_rail_neg_sp_kpa_{30.0};
  bool   gen_has_result_{false};

  int log_channel_id_{-1};
  std::ofstream log_file_;

  std::vector<double> prev_vol_m3_;
  std::set<int> active_channels_;

  std::chrono::steady_clock::time_point start_time_;
  double elapsed_time_sec_ = 0.0;

  // Sensor zero-calibration at startup
  static constexpr int ZERO_SAMPLES = 250;   // ~0.5 sec at 500 Hz
  // 0점 재보정이 yaml 기준에서 얼마나 벗어났는지 알리기 위해 원본을 보관한다.
  std::array<double, NUM_CAN_BOARDS> yaml_offset_{};
  double zero_tolerance_kpa_{8.0};
  // 액추에이터 미연결이면 엔코더도 미연결이다 — 각도를 0° 로 고정한다.
  bool encoder_zero_when_disconnected_{true};

  // ── 실측 제어 주기 ──────────────────────────────────────────────────
  // 제어 루프는 board/sensors 도착에 물려 돈다. 컨트롤러가 그 속도를 못 따라가면
  // 실제 틱 간격이 period_ms 보다 길어지는데, 예전에는 dt 를 **항상 period_ms 로**
  // 썼다 (실기 계측: 가정 2.0 ms, 실측 3.09 ms — 1.55배). 그 괴리는 특히
  // advance_valve_estimate 가 밸브 상태를 실시간 대비 1.55배 느리게 전진시켜,
  // 모델이 밸브를 굼뜨다고 보고 과도하게 명령하게 만든다 (첫스텝 포화 99%).
  // 실측 간격을 EMA 로 잡아 dt 로 쓴다. 스파이크는 클램프로 막는다.
  bool   use_measured_dt_{true};
  double dt_meas_sec_{-1.0};
  std::chrono::steady_clock::time_point last_tick_time_{};
  double dt_ctrl_sec_{0.002};        // 이번 틱에 실제로 쓰는 dt
  // ── 실행 중 켜고 끌 수 있는 보강 ──────────────────────────────────────
  // 파라미터 콜백이 갱신하고, on_timer 가 매 틱 각 MPC 의 cfg_.aug 로 밀어 넣는다.
  // 값 자체는 콜백 스레드와 제어 스레드가 함께 만지므로 뮤텍스로 보호한다.
  ControlAug aug_{};
  std::mutex aug_mtx_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr aug_cb_;
  void declare_aug_params();
  void push_aug_to_mpcs();
  bool   sensor_zeroed_{true};   // true = use YAML offsets directly (no auto-calib at startup)
  int    sensor_zero_tick_{0};
  std::array<double, NUM_CAN_BOARDS> sensor_zero_sum_{};
  std::array<int,    NUM_CAN_BOARDS> sensor_zero_cnt_{};

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_calib_srv_;

  // Over-pressure safety: positive channels only
  // Hysteresis: latch ON at >= limit, release only when P < (limit - hysteresis_kpa)
  double pressure_safety_limit_kpa_{170.0};
  double pressure_safety_hysteresis_kpa_{10.0};
  std::array<bool, 12> safety_latched_{};   // per positive channel (gid 0..11)
};
