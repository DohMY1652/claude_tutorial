#include "Controller.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <set>
#include <fstream>

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

using std::placeholders::_1;
using namespace std::chrono_literals;

// ================================
// RefTcpServer
// ================================
RefTcpServer::RefTcpServer(const Config& cfg, Callback cb)
: cfg_(cfg), cb_(std::move(cb))
{
#ifdef __linux__
    if (cfg_.enable)
        th_ = std::thread([this](){ run_(); });
#endif
}

RefTcpServer::~RefTcpServer()
{
    stop_.store(true);
    int sfd = server_fd_.exchange(-1);
    int cfd = client_fd_.exchange(-1);
#ifdef __linux__
    if (sfd >= 0) ::close(sfd);
    if (cfd >= 0) ::close(cfd);
#endif
    if (th_.joinable()) th_.join();
}

void RefTcpServer::run_()
{
#ifndef __linux__
    return;
#else
    int sfd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sfd < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("RefTcpServer"), "socket() failed");
        return;
    }
    server_fd_.store(sfd);

    int opt = 1;
    ::setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons((uint16_t)cfg_.port);

    if (::bind(sfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("RefTcpServer"), "bind() failed on port %d", cfg_.port);
        ::close(sfd); server_fd_.store(-1);
        return;
    }
    ::listen(sfd, 1);
    RCLCPP_INFO(rclcpp::get_logger("RefTcpServer"),
                "Listening for refs on port %d  [%d doubles per message]",
                cfg_.port, cfg_.num_values);

    while (!stop_.load()) {
        fd_set fds; FD_ZERO(&fds); FD_SET(sfd, &fds);
        struct timeval tv{1, 0};
        if (::select(sfd + 1, &fds, nullptr, nullptr, &tv) <= 0) continue;

        sockaddr_in cli{}; socklen_t cli_len = sizeof(cli);
        int cfd = ::accept(sfd, (struct sockaddr*)&cli, &cli_len);
        if (cfd < 0) continue;
        client_fd_.store(cfd);

        // 1s recv timeout so the inner loop can check stop_
        struct timeval rtv{1, 0};
        ::setsockopt(cfd, SOL_SOCKET, SO_RCVTIMEO, &rtv, sizeof(rtv));
        RCLCPP_INFO(rclcpp::get_logger("RefTcpServer"), "Client connected.");

        const size_t MSG = (size_t)cfg_.num_values * sizeof(double);
        std::vector<uint8_t> buf(MSG);
        bool ok = true;

        while (!stop_.load() && ok) {
            size_t total = 0;
            while (total < MSG && !stop_.load()) {
                ssize_t n = ::recv(cfd, buf.data() + total, MSG - total, 0);
                if (n == 0) { ok = false; break; }
                if (n < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
                    ok = false; break;
                }
                total += (size_t)n;
            }
            if (!ok || total < MSG) break;

            std::vector<double> vals((size_t)cfg_.num_values);
            for (size_t i = 0; i < vals.size(); ++i)
                std::memcpy(&vals[i], buf.data() + i * sizeof(double), sizeof(double));
            cb_(vals);
        }

        ::close(cfd); client_fd_.store(-1);
        RCLCPP_INFO(rclcpp::get_logger("RefTcpServer"), "Client disconnected.");
    }
    ::close(sfd); server_fd_.store(-1);
#endif
}

// yaml 파라미터를 읽되, 없으면 기본값을 쓴다.
//
// 주의: yaml 에 소수점 없이 적힌 값(예: pos_sp_max_kpa: 150)은 int 로 파싱되어
// declare_parameter<double>() 이 InvalidParameterTypeException 을 던진다. 예전 구현은
// 그 예외를 삼키고 **기본값을 조용히 반환**했다 — yaml 을 고쳐도 반영되지 않는 함정이라
// 튜닝 중에 실제로 물렸다. 이제 예외 후 get_parameter 로 실제 타입에 맞춰 읽고,
// 산술 타입이면 정수 → 실수 변환까지 구제한다.
// (CanBridge::declare_double_flexible / VirtualPowerpack::gpd 와 같은 처리)
template <typename T>
static T get_param_or(rclcpp::Node* node, const std::string& name, const T& defv) {
  if (!node->has_parameter(name)) {
    try {
      return node->declare_parameter<T>(name, defv);
    } catch (...) {
      // 타입 불일치 — 아래에서 읽는다
    }
  }
  T out = defv;
  if (node->get_parameter(name, out)) return out;

  if constexpr (std::is_arithmetic_v<T>) {
    rclcpp::Parameter p;
    if (node->get_parameter(name, p)) {
      if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        return static_cast<T>(p.as_int());
      if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        return static_cast<T>(p.as_double());
    }
  }
  return defv;
}

// ================================
// ThreadPool
// ================================
ThreadPool::ThreadPool(size_t num_threads, const std::vector<int>& pin_cpus)
: pin_cpus_(pin_cpus)
{
  workers_.reserve(num_threads);
  for (size_t i = 0; i < num_threads; ++i) {
    workers_.emplace_back([this, i](){
#ifdef __linux__
      if (!pin_cpus_.empty()) {
        int cpu = pin_cpus_[i % pin_cpus_.size()];
        cpu_set_t set; CPU_ZERO(&set); CPU_SET(cpu, &set);
        pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
      }
#endif
      while (true) {
        std::function<void()> task;
        {
          std::unique_lock<std::mutex> lk(mtx_);
          cv_.wait(lk, [&]{ return stop_.load() || !queue_.empty(); });
          if (stop_.load()) return;
          task = std::move(queue_.front()); queue_.pop();
        }
        if (task) task();
      }
    });
  }
}

ThreadPool::~ThreadPool() {
  stop_.store(true);
  cv_.notify_all();
  for (auto& t : workers_) if (t.joinable()) t.join();
}

void ThreadPool::enqueue(std::function<void()> fn) {
  {
    std::lock_guard<std::mutex> lk(mtx_);
    queue_.push(std::move(fn));
  }
  cv_.notify_one();
}

void ThreadPool::run_batch_and_wait(std::vector<std::function<void()>>& tasks) {
  if (tasks.empty()) return;
  std::atomic<size_t> done{0};
  const size_t N = tasks.size();
  for (auto& f : tasks) {
    enqueue([&done, f](){ f(); done.fetch_add(1, std::memory_order_relaxed); });
  }
  while (done.load(std::memory_order_acquire) < N) {
    std::this_thread::yield();
  }
}

// ================================
// AcadosMpc
// ================================
AcadosMpc::AcadosMpc(const Config& cfg) : cfg_(cfg) {
  P_ref_.assign(cfg_.NP, cfg_.ref_value);
  A_seq_.assign(cfg_.NP, cfg_.A_lin);
  Eigen::RowVector3f b_row = Eigen::Map<const Eigen::RowVector3f>(cfg_.B_lin.data());
  B_seq_.assign(cfg_.NP, b_row);

  const int Nx_ = cfg_.n_x * cfg_.NP;
  const int Nu_ = cfg_.n_u * cfg_.NP;

  Q_.setZero(Nx_, Nx_);
  R_.setZero(Nu_, Nu_);
  for (int i = 0; i < Nx_; ++i) Q_(i, i) = cfg_.Q_value;
  for (int i = 0; i < Nu_; ++i) R_(i, i) = cfg_.R_value;

  // 매 solve() 호출 시 heap alloc 방지용 pre-alloc
  Pmat_.setZero(Nu_, Nu_);
  qvec_.setZero(Nu_);
  Acon_.setZero(Nu_, Nu_);
  LL_.setZero(Nu_);
  UL_.setZero(Nu_);
  S_bar_.setZero(Nx_, Nu_);
  T_bar_.setZero(Nx_, cfg_.n_x);
  x0_mpc_.setZero(cfg_.n_x);
  Xref_mpc_.setZero(cfg_.NP);
  qtmp_.setZero(Nx_);
  solution_.setZero(Nu_);

  vol_dot_buffer_.clear();
  for(size_t i=0; i<vol_dot_window_size_; ++i) {
      vol_dot_buffer_.push_back(0.0f);
  }

  // ── MPPI 경로 구성 ─────────────────────────────────────────────────────
  // 밸브별 13-parameter 는 build_mpcs 가 cfg_.pv 에 이미 채웠다 (단일 출처).
  if (cfg_.use_mppi) {
    mppi_pv_ = cfg_.pv;
    for (auto& t : mppi_pv_) t.finalize();

    mppi::Params pr;
    pr.K        = cfg_.mppi_samples;
    pr.NP       = (cfg_.mppi_np    > 0)   ? cfg_.mppi_np    : cfg_.NP;
    pr.Ts       = (cfg_.mppi_ts_s  > 0.f) ? cfg_.mppi_ts_s  : cfg_.Ts;
    pr.substeps = cfg_.mppi_substeps;
    pr.lambda   = cfg_.mppi_lambda;
    pr.sigma_pct  = cfg_.mppi_sigma_pct;
    pr.sigma_explore_pct = cfg_.mppi_sigma_explore_pct;
    pr.explore_frac      = cfg_.mppi_explore_frac;
    pr.noise_beta = cfg_.mppi_noise_beta;
    pr.w_track  = (cfg_.mppi_w_track  >= 0.f) ? cfg_.mppi_w_track  : cfg_.Q_value;
    pr.w_effort = (cfg_.mppi_w_effort >= 0.f) ? cfg_.mppi_w_effort : cfg_.R_value;
    pr.w_du     = cfg_.mppi_w_du;
    pr.track_scale_kpa = cfg_.mppi_track_scale_kpa;
    pr.terminal_mult   = cfg_.mppi_terminal_mult;
    pr.du_min = -cfg_.mppi_du_limit_pct;
    pr.du_max = +cfg_.mppi_du_limit_pct;

    // 시드는 채널마다 다르되 **고정**이다 (하네스가 이미 비결정론적이므로).
    mppi_ = std::make_unique<mppi::Solver>(mppi_pv_, pr, (uint32_t)(0x9E37u + cfg_.global_id));
  }
}


void AcadosMpc::set_qp_solver(std::shared_ptr<QP> qp) { qp_ = std::move(qp); }

void AcadosMpc::update_linearization(float /*x_ref*/,
                                     const Eigen::RowVector3f& u_ref)
{
  const float P_now   = current_P_now_;
  const float P_micro = current_P_micro_;
  const float P_macro = current_P_macro_;
  const float P_atm   = current_P_atm_; 

  const double lpm2kgps  = 0.0002155;
  const double Rgas      = 287.0;
  const double TempK     = 293.15;
  const double Volume    = std::max(1e-12, (double)cfg_.volume_m3);


  // 정적 유량 [LPM] — **밸브별 파라미터**를 쓴다 (mppi 커널 재사용).
  // 예전에는 채널 공용 값을 써서 피팅한 atm/macro 값이 선형화에 반영되지 않았다.
  auto Q_static_fn = [&](int j, double u_pct, double Pin, double Pout, double z_val) -> double {
    return (double)mppi::q_static(cfg_.pv[(size_t)j], (float)std::clamp(u_pct, 0.0, 100.0),
                                  (float)Pin, (float)Pout, (float)z_val);
  };

  // Numerical Jacobian of Q_static → [dQ/du_pct, dQ/dPin, dQ/dPout] * scale
  // Matches the original calc_rounds return convention: [round_input, round_pin, round_pout]
  auto calc_rounds = [&](int j, double input, double Pin, double Pout, double z_val)
  {
    input = std::clamp(input, 0.0, 100.0);
    const double Q0 = Q_static_fn(j, input, Pin, Pout, z_val);

    constexpr double du = 0.5, dP = 0.5;
    const double dQ_du   = (Q_static_fn(j, std::min(input+du, 100.0), Pin,    Pout,    z_val) - Q0) / du;
    const double dQ_dPin = (Q_static_fn(j, input, Pin + dP, Pout,    z_val) - Q0) / dP;
    const double dQ_dPout= (Q_static_fn(j, input, Pin,    Pout + dP, z_val) - Q0) / dP;

    const double scale = (Rgas * TempK / Volume) * lpm2kgps;
    return std::array<double,3>{dQ_du*scale, dQ_dPin*scale, dQ_dPout*scale};
  };

  // Ejector (macro valve in negative channel) uses same 13-var model,
  // but P_limit provides a minimum pressure for flow to occur.
  auto ejector_calc_rounds = [&](double input, double P_chamber)
  {
    const double P_limit = (double)cfg_.ejector_p_limit;
    if (P_chamber <= P_limit) return std::array<double,3>{0.0, 0.0, 0.0};
    // Treat ejector suction as flow from P_chamber to P_limit
    return calc_rounds(mppi::V_MACRO, input, P_chamber, P_limit, z_macro_);
  };

  const double u_mi = std::clamp((double)u_ref(0), 0.0, 100.0);
  const double u_ma = std::clamp((double)u_ref(1), 0.0, 100.0);
  const double u_at = std::clamp((double)u_ref(2), 0.0, 100.0);
  const double leak_u_pos = (double)cfg_.leakage_u_pos;
  const double leak_u_neg = (double)cfg_.leakage_u_neg;

  double A_scalar = 0.0;
  Eigen::RowVector3f B_row; B_row.setZero();

  if (cfg_.is_positive) {
    auto mi = calc_rounds(mppi::V_MICRO, u_mi, P_micro, P_now,  z_micro_);
    auto ma = calc_rounds(mppi::V_MACRO, u_ma, P_macro, P_now,  z_macro_);
    auto at = calc_rounds(mppi::V_ATM,   u_at, P_now,   P_atm,  z_atm_);
    auto lk = calc_rounds(mppi::V_ATM,   leak_u_pos, P_now, P_atm, z_atm_);

    const double tmp_A = mi[2] + ma[2] - at[1] - lk[1];
    const double b0 =  mi[0];
    const double b1 =  ma[0];
    const double b2 = -at[0];

    A_scalar = (float)tmp_A;
    B_row << (float)b0, (float)b1, (float)b2;

  } else {
    auto at = calc_rounds(mppi::V_ATM,   u_at, P_atm,  P_now,  z_atm_);
    auto mi = calc_rounds(mppi::V_MICRO, u_mi, P_now,  P_micro, z_micro_);
    auto ma = ejector_calc_rounds(u_ma, P_now);
    auto lk = calc_rounds(mppi::V_ATM,   leak_u_neg, P_atm, P_now, z_atm_);

    const double tmp_A = at[2] - mi[1] - ma[1] + lk[2];
    const double b0 = -mi[0];
    const double b1 = -ma[0];
    const double b2 =  at[0];

    A_scalar = (float)tmp_A;
    B_row << (float)b0, (float)b1, (float)b2;
  }

  A_seq_.assign(cfg_.NP, A_scalar);
  B_seq_.assign(cfg_.NP, B_row);
}

void AcadosMpc::set_AB_sequences(const std::vector<float>& A_seq,
                                 const std::vector<Eigen::RowVector3f>& B_seq) {
  const int NP = cfg_.NP;
  if ((int)A_seq.size() != NP || (int)B_seq.size() != NP) return;
  A_seq_ = A_seq;
  B_seq_ = B_seq;
}

void AcadosMpc::set_AB_constant(float A_scalar, const Eigen::RowVector3f& B_row) {
  A_seq_.assign(cfg_.NP, A_scalar);
  B_seq_.assign(cfg_.NP, B_row);
}


std::array<float,3> AcadosMpc::compute_input_reference(float P_now, float P_micro, float P_macro, float P_macro_neg, float dt_sec, float current_time_sec) {
  const double lpm2kgps  = 0.0002155;
  const double Rgas      = 287.0;
  const double TempK     = 293.15;

  const double current_vol = std::max(1e-12, (double)cfg_.volume_m3);
  const double prev_vol    = std::max(1e-12, (double)cfg_.prev_vol_m3);

  // Update Bouc-Wen hysteresis states from last commanded currents
  // Bouc-Wen 히스테리시스도 **밸브별 파라미터**로 갱신한다 (A_bw/beta/gamma/I_MAX 가
  // 밸브마다 다르다). 예전에는 채널 공용 값을 써서 피팅 결과가 반영되지 않았다.
  // mppi::step_bw 와 **같은 적분**이어야 한다 — dI 를 수축조건까지 쪼갠다.
  // (쪼개지 않으면 beta_bw·|dI| ≫ 1 에서 z 가 ±1e6 로 발산한다. Mppi.cpp 주석 참조.)
  auto update_bw = [&](int j, double& z, double& prev_I, int& dir, double u_pct) {
    const auto& p = cfg_.pv[(size_t)j];
    const double I     = u_pct / 100.0 * (double)p.I_MAX;
    const double dI    = I - prev_I;
    const double abs_dI = std::abs(dI);
    const double rate  = ((double)p.beta_bw + (double)p.gamma_bw) * abs_dI;
    const int    n     = (rate > 0.25) ? std::min(256, (int)std::ceil(rate / 0.25)) : 1;
    const double ddI = dI / n, abs_ddI = abs_dI / n;
    for (int i = 0; i < n; ++i) {
      z += (double)p.A_bw * ddI
         - (double)p.beta_bw  * abs_ddI * z
         - (double)p.gamma_bw * ddI * std::abs(z);
      z = std::clamp(z, -1e6, 1e6);
    }
    if      (dI >  1e-4) dir = 1;
    else if (dI < -1e-4) dir = 0;
    prev_I = I;
  };
  update_bw(mppi::V_MICRO, z_micro_, prev_I_micro_, dir_micro_, (double)last_u3_[0]);
  update_bw(mppi::V_ATM,   z_atm_,   prev_I_atm_,   dir_atm_,   (double)last_u3_[2]);
  update_bw(mppi::V_MACRO, z_macro_, prev_I_macro_, dir_macro_, (double)last_u3_[1]);

  // Compressible Phi helper (kappa=1.4)
  auto get_phi_ff = [](double Pin, double Pout) -> double {
    if (Pin < 1e-9 || Pout >= Pin) return 0.0;
    constexpr double kappa = 1.4;
    const double Pr  = std::clamp(Pout / Pin, 0.0, 1.0);
    const double Pcr = std::pow(2.0/(kappa+1.0), kappa/(kappa-1.0));
    if (Pr <= Pcr)
      return std::sqrt(kappa * std::pow(2.0/(kappa+1.0), (kappa+1.0)/(kappa-1.0)));
    return std::sqrt(2.0*kappa/(kappa-1.0)) * std::sqrt(std::max(0.0,
      std::pow(Pr, 2.0/kappa) - std::pow(Pr, (kappa+1.0)/kappa)));
  };

  // 역모델은 `mppi::valve_invert` / `mppi::u_of_area` / `PlantParams::u_crack` 을 쓴다
  // — 예전에는 여기 람다로 복제돼 있었고, 그래서 **밸브별 파라미터를 쓸 수 없었다.**
  // 이제 밸브 인덱스(0=micro, 1=macro, 2=atm)로 각자의 13-parameter 를 적용한다.
  // C_p·Pin 항 때문에 상류 압력이 높으면 필요 전류가 낮아진다 (압력이 스풀을 돕는다).
  auto valve_invert = [&](int j, double Q_req, double Pin, double Pout, double z_val) -> float {
    return mppi::valve_invert(cfg_.pv[(size_t)j], (float)Q_req, (float)Pin, (float)Pout,
                              (float)z_val);
  };
  // 크래킹 임계 [%] — 이 명령 이하에서는 스풀이 들리지 않아 유량이 0 이다.
  // 실측(50% 부근)과 일치한다: 351 kPa abs 레일에서 약 52% 다.
  auto u_crack = [&](int j, double Pin, double z_val) {
    return cfg_.pv[(size_t)j].u_crack((float)Pin, (float)z_val);
  };
  (void)get_phi_ff;

  // ── 보강 ① 유량 이득 적응 ───────────────────────────────────────────
  // 직전 틱에 실제로 인가한 명령의 모델 유량(q_model_last_)과, 그 사이 챔버가 실제로
  // 보인 유량을 비교한다. 챔버 자체가 유량계다: q = dP/dt·V/(R·T).
  // 부피 추정이나 오리피스 환산이 틀려도 이 비가 그만큼을 흡수한다.
  // 측정 유량은 **필터 전 생값**으로 낸다. 컨트롤러 LPF 를 통과한 값으로 미분하면
  // 과도 구간에서 dP/dt 가 지연·축소되고, 그 손실이 그대로 k 의 하향 편향이 된다
  // (시뮬에서 모델=플랜트라 참값이 1.0 인데 0.52 까지 내려갔다).
  const float p_for_rate = cfg_.mppi_raw_state ? current_P_now_raw_ : current_P_now_raw_;
  if (cfg_.aug.adapt_gain && p_prev_meas_ > 0.0f && dt_sec > 1e-6f &&
      std::abs(q_model_last_) > cfg_.aug.adapt_min_flow_lpm) {
    const float dpdt = (p_for_rate - p_prev_meas_) / dt_sec;            // [kPa/s]
    const float q_meas = dpdt * 1000.0f * current_vol / (Rgas * TempK) / lpm2kgps;
    // **틱마다 q_meas/q_model 을 나누면 안 된다.** 생값 미분은 0.25 kPa 양자화에서
    // ±125 kPa/s 잡음이라 비가 폭넓게 흩어지고, 클램프·게이트와 겹쳐 편향이 생긴다
    // (계측: 필터값으로는 0.52, 생값 틱별 비로는 1.56 — 참값 1.0 을 양쪽으로 빗나갔다).
    // 창을 모아 최소자승 이득 k = Σ(q_meas·q_model)/Σ(q_model²) 로 구한다.
    // 분모에 잡음이 없으므로 이 추정은 편향되지 않는다.
    if (std::isfinite(q_meas)) {
      adapt_sxy_ += (double)q_meas * (double)q_model_last_;
      adapt_sxx_ += (double)q_model_last_ * (double)q_model_last_;
      ++adapt_n_;
    }
    if (adapt_n_ >= cfg_.aug.adapt_window && adapt_sxx_ > 1e-9) {
      const float k_ls = (float)(adapt_sxy_ / adapt_sxx_);
      if (std::isfinite(k_ls) && k_ls > 0.0f) {
        k_flow_ += cfg_.aug.adapt_rate * (k_ls - k_flow_);
        k_flow_ = std::clamp(k_flow_, cfg_.aug.gain_min, cfg_.aug.gain_max);
      }
      adapt_sxy_ = adapt_sxx_ = 0.0; adapt_n_ = 0;
    }
  }
  if (!cfg_.aug.adapt_gain) k_flow_ = 1.0f;   // 끄면 즉시 모델 그대로로 되돌린다
  // 측정 압력 변화율 — 크래킹 하한을 걸지 말지 판단하는 데 쓴다.
  // 생값 미분은 0.25 kPa 양자화에서 ±125 kPa/s 잡음이라 반드시 걸러야 한다.
  if (p_prev_meas_ > 0.0f && dt_sec > 1e-6f) {
    const float raw = (p_for_rate - p_prev_meas_) / dt_sec;
    if (std::isfinite(raw)) {
      const float a = dt_sec / (0.10f + dt_sec);          // τ = 100 ms
      dpdt_f_ += a * (raw - dpdt_f_);
    }
  }
  p_prev_meas_ = p_for_rate;

  // ── 보강 ② 오프셋 프리 (출력 외란 추정) ─────────────────────────────
  // 정상상태 오차를 레퍼런스 쪽으로 흡수한다. 데드밴드는 센서 분해능(0.25 kPa)보다
  // 크게 두어 잡음을 적분하지 않는다.
  if (cfg_.aug.offset_free) {
    const float e0 = cfg_.ref_value - P_now;
    // ── 안티와인드업 ──────────────────────────────────────────────────
    // 적분은 **컨트롤러가 실제로 더 밀 수 있을 때만** 의미가 있다. 두 경우를 막는다:
    //  · 밸브가 이미 포화 — 더 요구해도 나갈 유량이 없다
    //  · 과압 세이프티 래치 — 명령이 무시되고 밸브가 강제 전개된다. 그 구간의 오차는
    //    컨트롤러 탓이 아닌데 적분하면, 래치가 풀린 뒤 감긴 만큼 과도하게 민다
    //    (계측: 이 보호 없이 d 가 한계 +29 kPa 까지 감겨 레퍼런스를 214 kPa 로 밀고
    //     세이프티가 225 샘플 재발동했다).
    const bool sat_up = cfg_.is_positive ? (last_u3_[0] >= 99.5f || last_u3_[1] >= 99.5f)
                                         : (last_u3_[2] >= 99.5f);
    const bool sat_dn = cfg_.is_positive ? (last_u3_[2] >= 99.5f)
                                         : (last_u3_[0] >= 99.5f || last_u3_[1] >= 99.5f);
    const bool blocked = safety_latched_ext_
                      || (e0 > 0.0f && sat_up) || (e0 < 0.0f && sat_dn);
    // 적분은 **정상상태 보정**이다 — 과도 구간에서 적분하면 그 자체가 제어기가 돼
    // 오버슈트를 만든다. 오차가 band 안(= 거의 다 왔다)일 때만 적분한다.
    // 속도는 **초당** 값이라 제어 주기(500 Hz)와 무관하다. 예전에는 틱당 0.01 이라
    // 실효 시상수가 0.2 s 였고, 오차 55 kPa 짜리 과도에서 0.1 초 만에 한계까지 감겼다.
    const bool in_band = std::abs(e0) > cfg_.aug.dist_deadband_kpa
                      && std::abs(e0) < cfg_.aug.dist_band_kpa;
    if (!blocked && in_band) {
      d_hat_ += cfg_.aug.dist_rate * e0 * dt_sec;
      d_hat_ = std::clamp(d_hat_, -cfg_.aug.dist_limit_kpa, cfg_.aug.dist_limit_kpa);
    }
    // 래치 중에는 감긴 것을 **되돌린다** — 래치는 "너무 높다"는 뜻이므로 d 를 줄인다.
    if (safety_latched_ext_ && d_hat_ > 0.0f)
      d_hat_ = std::max(0.0f, d_hat_ - cfg_.aug.dist_rate * cfg_.aug.dist_limit_kpa);
  } else {
    d_hat_ = 0.0f;
  }

  const float Pref = cfg_.ref_value + d_hat_;
  ref_eff_ = Pref;                       // solve() 의 MPPI 도 같은 목표를 본다
  float err = Pref - P_now;

  // 하드 데드밴드는 여기 있었지만 제거했다 — solve() 의 명령 테이퍼로 대체한다.
  // (데드밴드는 그 자체가 릴레이라서 밸브 지연과 만나면 리밋사이클을 만든다)

  float raw_vol_dot = float((current_vol - prev_vol) / dt_sec);

  vol_dot_buffer_.push_back(raw_vol_dot);
  if (vol_dot_buffer_.size() > vol_dot_window_size_) {
      vol_dot_buffer_.pop_front(); 
  }

  float sum_vol_dot = 0.0f;
  for (float val : vol_dot_buffer_) {
      sum_vol_dot += val;
  }
  
  float vol_dot = sum_vol_dot / (float)vol_dot_buffer_.size();
  vol_dot_est_ = vol_dot;          // MPPI 롤아웃의 외생 입력으로 재사용

  float target_time_constant = cfg_.target_time_constant;
  if (target_time_constant <= 0.001f) target_time_constant = 0.2f;
  
  const float P_abs_atm = current_P_atm_; 
  
  // P_dot 계산 (Feedforward)
  float P_dot = (Pref - P_now) * 1000 / target_time_constant; //[Pa/sec]

  float m_dot_pressure =  P_dot  * current_vol / (Rgas * TempK) / lpm2kgps; //[LPM]
  float m_dot_volume = P_now * 1000 * vol_dot / (Rgas * TempK) / lpm2kgps; //[LPM]
  
  // 적분기(Integral) 업데이트
  constexpr float integral_limit = 1000.0f;
  // 부호가 바뀔 때 적분기를 0 으로 **리셋하지 않는다.**
  //
  // 예전에는 오차 부호가 바뀌는 순간 통째로 0 으로 지웠다. 이 밸브는 크래킹
  // 아래에서 유량이 0 이라 적분기가 크래킹까지 기어오르는 데 6 초쯤 걸리는데,
  // 목표를 지나치는 순간 그 6 초치를 전부 버리고 다시 0 에서 시작한다. 그래서
  // 평균이 목표 위에 앉는다 (시뮬: trim 8.9 → 0.0, 지령 54.3% → 44.9%, 오차가
  // +1.0 kPa 에 고착). 지금은 보통의 적분기처럼 오차 부호를 따라 자연히 줄어든다.
  // 폭주는 ki_u_limit_pct(기여분 상한)와 아래 anti-windup 이 막는다.
  // 데드존 인식 적분 — 압력이 안 움직이면 빨리, 움직이기 시작하면 멈춘다.
  //
  // 이 밸브는 크래킹 아래에서 유량이 정확히 0 이라, 크래킹 위치를 조금만 낮게
  // 잡아도 피드포워드가 데드존에 앉는다. 그때 적분기가 통상 속도로 기어오르면
  // 수 초가 걸린다 (실기 20260829_152528: ch1 이 지령 43~45% 에서 1.4 초 정체 후
  // 1.7 초에 걸쳐 62.9% 까지 올라가고서야 열렸다 — 총 3.2 초 지연).
  // 반대로 크래킹을 높게 잡으면 피드포워드가 이미 충분한데 적분기가 계속 쌓여
  // 오버슛한다 (같은 실기 ch1: 열리는 순간 0.18 초에 +23 kPa, 2 Hz 진동).
  //
  // 크래킹 위치를 정확히 아는 것으로 이 둘을 동시에 잡기는 어렵다 — 진동하는
  // 채널은 지령을 0.3 초도 유지하지 않아 폐루프 로그로 측정 자체가 안 된다.
  // 대신 **압력이 실제로 반응하는지**를 보고 적분 속도를 바꾼다:
  //   · 원하는 방향으로 충분히 움직이는 중  → 적분 멈춤 (피드포워드가 일하고 있다)
  //   · 안 움직임(데드존)                   → 부스트 배속으로 빠르게 통과
  // 크래킹을 몰라도 두 경우가 모두 처리된다.
  // 기본은 **꺼짐**이다 (integ_hold_rate_kpas <= 0). 시뮬에서는 모델과 플랜트가
  // 가까워 데드존이 짧고, 그 조건에서는 정지·부스트가 오히려 정착을 늦춘다
  // (플랜트 +3%p 뻑뻑: 정착 0.69 → 1.91 s, 부스트 6 배면 5.73 s).
  // 실기의 긴 데드존(3.2 초)에서만 이득이 있을 수 있으므로 실기 검증 후 켤 것.
  const bool hold_on = (cfg_.integ_hold_rate_kpas > 0.0f);
  const bool responding = hold_on && (want_sign_ * dpdt_f_ > cfg_.integ_hold_rate_kpas);
  const float ki_scale = responding ? 0.0f : cfg_.integ_deadzone_boost;
  if (cfg_.is_positive) {
    pos_error_integral_ += ki_scale * err * dt_sec;
    pos_error_integral_ = std::clamp(pos_error_integral_, -integral_limit, integral_limit);
  } else {
    neg_error_integral_ -= ki_scale * err * dt_sec;
    neg_error_integral_ = std::clamp(neg_error_integral_, -integral_limit, integral_limit);
  }
  
  // 적분 **기여분** 을 지령 몇 %p 로 묶는다.
  //
  // 이 밸브는 비례대역이 5.2%p 뿐이고 크래킹 아래에서는 유량이 정확히 0 이다.
  // 그 구간에서 적분기는 아무 반응도 못 받은 채 계속 쌓이다가, 밸브가 열리는
  // 순간 718 kPa/s 로 한꺼번에 쏟아진다. 실제로 플랜트를 모델보다 8% 뻑뻑하게
  // 두고 시험하니 오버슛이 +42~+100 kPa 였고 끝내 정착하지 못했다.
  // 적분기가 해야 할 일은 "모델이 빗나간 만큼(≈8%p) 을 메우는 것" 이지
  // 밸브를 100% 까지 미는 것이 아니다. 기여분을 묶으면 그 역할만 남는다.
  // (u≥100 에서만 멈추던 기존 anti-windup 은 크래킹 아래에서는 절대 걸리지 않는다.)
  // 지령 트림은 이제 **크래킹 위치 오차**만 담당한다 (유량 오차는 위의 q_gain).
  // 비례대역(≈6%p)의 절반을 넘으면 그것만으로 밸브를 열고 닫을 수 있어
  // 릴레이가 된다.
  ki_flow_ = cfg_.ki_flow;
  const float ki_u_lim = cfg_.ki_u_limit_pct;
  // 밸브는 한 방향으로만 흐른다. 자기 방향의 오차가 쌓였을 때만 열고, 반대
  // 부호일 때는 0 이다 (예전의 std::abs() 는 부호 리셋이 있을 때만 성립했다).
  auto ki_term = [ki_u_lim](float ki, float integ) {
    return std::clamp(std::max(0.0f, ki * integ), 0.0f, ki_u_lim);
  };
  const float ki_mi = cfg_.is_positive ? cfg_.pos_ki_micro : cfg_.neg_ki_micro;
  const float ki_ma = cfg_.is_positive ? cfg_.pos_ki_macro : cfg_.neg_ki_macro;
  const float ki_at = cfg_.is_positive ? cfg_.pos_ki_atm   : cfg_.neg_ki_atm;
  
  float u_mi_req = 0.f, u_ma_req = 0.f, u_at_req = 0.f;
  u_trim_ = {0.f, 0.f, 0.f};

  // macro 분담은 **판정이 아니라 나눗셈**이다 — MATLAB update_sources 와 같은 규칙:
  //     m_fill  = min(요구, 레일이 낼 수 있는 최대)
  //     m_boost = max(0, 요구 − 레일 최대)
  // 레일이 감당하면 m_boost 가 0 이라 macro 는 저절로 닫히고, 모자라는 순간 모자란
  // 만큼만 열린다. "열지 말지"를 정하는 임계값이 필요 없다 — 예전의
  // macro_gate_frac(0.02) · macro_micro_sat_pct(100) · macro_threshold(50 kPa) 는
  // 전부 이 한 줄이 대신한다.
  //
  // 이전 코드는 게이트가 열리면 micro 와 macro **양쪽에 전량 Q_req 를** 요구했다.
  // 실제 유량이 의도의 두 배가 되고 macro 상류는 탱크(≈670 kPa)라, 목표 178 kPa 인
  // 채널이 550 kPa 까지 올라가 과압 세이프티가 반복 래치됐다 (HANDOFF 3-1).
  auto split_demand = [&](int j_rail, double q_req, double Pin, double Pout, double z,
                          double& q_rail, double& q_boost) {
    // 용량도 같은 이득으로 본다 — 이득이 분자·분모에 함께 들어가야 분배 비율이
    // 이득과 무관해진다 (macro 로 새는 것을 막는다).
    const double cap = (double)mppi::q_static(cfg_.pv[(size_t)j_rail], 100.0f,
                                              (float)Pin, (float)Pout, (float)z)
                     / std::max(1e-3f, k_flow_);
    q_rail  = std::min(q_req, cap);
    q_boost = std::max(0.0, q_req - cap);
  };
 
  // Feedforward: 역모델로 필요한 u_pct 계산
  // 모델이 실제보다 k_flow_ 배 많이 흘린다고 추정되면 그만큼 **덜** 요구해야
  // 실제 유량이 목표가 된다. k_flow_=1 이면 기존과 동일하다.
  // 적분 보정을 **유량**에 건다 (지령이 아니라).
  //
  // 예전에는 적분기가 지령에 상수 %p 를 더했다. 이 밸브는 비례대역이 6.2%p 라
  // 그 방식은 오차 크기와 무관하게 밸브를 활짝 열어 버린다 — 오차 1 kPa 이든
  // 15 kPa 이든 같은 +15%p 가 붙는다.
  //   실기 20260828_184156 (6축): 모델이 요구한 지령은 38.5%(면적 2% 개방)인데
  //   실제 지령이 53% 였다. 차이 15%p 는 트림이 상한에 포화한 것이고, 53% 는
  //   그 밸브에서 사실상 완전 개방이다 — 의도의 48 배 유량이다. 그 결과
  //   v1↔v2 가 3 Hz 로 왕복하며 챔버가 p-p 15~40 kPa 로 진동했다.
  //
  // 유량에 걸면 보정이 수요에 비례한다. 오차가 작으면 요구 유량도 작고 지령도
  // 크래킹 근처에 머문다. 모델이 실제보다 많이 흘린다고 보면 그만큼 더 요구해
  // 역모델이 알아서 더 큰 지령을 낸다 — 비선형은 역모델이 처리한다.
  //
  // 크래킹 위치 자체가 틀린 경우(유량이 아예 0)는 이 배율로 못 고친다.
  // 그것은 finish() 의 크래킹 하한이 담당한다.
  // 부호: 이번 틱에 여는 밸브의 **작용 방향**에 맞춰야 한다.
  //   양압 micro/macro(채움) 은 pos 적분이 양수일 때 더 흘려야 하고,
  //   양압 atm(배기) 은 pos 적분이 음수일 때 더 흘려야 한다. 음압도 대칭이다.
  const double m_dot_sum = (double)(m_dot_pressure + m_dot_volume);
  const bool q_via_atm = cfg_.is_positive ? !(m_dot_sum > 0.0) : !(m_dot_sum < 0.0);
  const float integ_dir = (q_via_atm ? -1.0f : +1.0f)
                        * (cfg_.is_positive ? (float)pos_error_integral_
                                            : (float)neg_error_integral_);
  const float q_gain = 1.0f + std::clamp(ki_flow_ * integ_dir,
                                         -cfg_.q_trim_limit, cfg_.q_trim_limit);
  q_trim_ = q_gain;
  const double Q_req = (double)std::abs(m_dot_pressure + m_dot_volume)
                     / std::max(1e-3f, k_flow_)
                     * (double)std::max(0.2f, q_gain);

  if (cfg_.is_positive) {
    // 양압 채널: micro=레일→챔버, macro=탱크→챔버, atm=챔버→대기
    u_crack_ = { u_crack(mppi::V_MICRO, (double)P_micro, z_micro_),
                 u_crack(mppi::V_MACRO, (double)P_macro, z_macro_),
                 u_crack(mppi::V_ATM,   (double)P_now,   z_atm_) };
    if ((m_dot_pressure + m_dot_volume) > 0.f) {
      double q_rail = 0.0, q_boost = 0.0;
      split_demand(mppi::V_MICRO, Q_req, (double)P_micro, (double)P_now, z_micro_,
                   q_rail, q_boost);
      u_mi_req = valve_invert(mppi::V_MICRO, q_rail, (double)P_micro, (double)P_now, z_micro_);
      u_trim_[0] = ki_term(ki_mi, pos_error_integral_);
      u_at_req = 0.f;
      u_ma_req = (q_boost > 0.0)
                 ? valve_invert(mppi::V_MACRO, q_boost, (double)P_macro, (double)P_now, z_macro_)
                 : 0.f;
      u_trim_[1] = (q_boost > 0.0) ? ki_term(ki_ma, pos_error_integral_) : 0.f;
    } else {
      u_mi_req = 0.f;
      u_ma_req = 0.f;
      u_at_req = valve_invert(mppi::V_ATM, Q_req, (double)P_now, (double)P_abs_atm, z_atm_);
      u_trim_[2] = ki_term(ki_at, -pos_error_integral_);
    }
  } else {
    // 음압 채널: micro=챔버→음압레일, macro=챔버→이젝터, atm=대기→챔버
    u_crack_ = { u_crack(mppi::V_MICRO, (double)P_now,     z_micro_),
                 u_crack(mppi::V_MACRO, (double)P_now,     z_macro_),
                 u_crack(mppi::V_ATM,   (double)P_abs_atm, z_atm_) };
    if ((m_dot_pressure + m_dot_volume) < 0.f) {
      double q_rail = 0.0, q_boost = 0.0;
      split_demand(mppi::V_MICRO, Q_req, (double)P_now, (double)P_micro, z_micro_,
                   q_rail, q_boost);
      u_mi_req = valve_invert(mppi::V_MICRO, q_rail, (double)P_now, (double)P_micro, z_micro_);
      u_trim_[0] = ki_term(ki_mi, neg_error_integral_);
      u_at_req = 0.f;
      u_ma_req = (q_boost > 0.0)
                 ? valve_invert(mppi::V_MACRO, q_boost, (double)P_now, (double)cfg_.ejector_p_limit, z_macro_)
                 : 0.f;
      u_trim_[1] = (q_boost > 0.0) ? ki_term(ki_ma, neg_error_integral_) : 0.f;
    } else {
      u_mi_req = 0.f;
      u_ma_req = 0.f;
      u_at_req = valve_invert(mppi::V_ATM, Q_req, (double)P_abs_atm, (double)P_now, z_atm_);
      u_trim_[2] = ki_term(ki_at, -neg_error_integral_);
    }
  }

  // 어느 밸브에 유량을 요구했는지, 그리고 그 요구가 압력을 **올리려는 것인지
  // 내리려는 것인지** 기록해 둔다. 크래킹 하한은 finish() 맨 끝에서 건다.
  //   양압 채널: micro·macro = 레일/탱크 → 챔버 (올림),  atm = 챔버 → 대기 (내림)
  //   음압 채널: micro·macro = 챔버 → 진공/이젝터 (내림), atm = 대기 → 챔버 (올림)
  u_want_ = { u_mi_req > 0.0f, u_ma_req > 0.0f, u_at_req > 0.0f };
  err_abs_ = std::abs(err);
  const bool via_atm = (u_at_req > 0.0f);
  want_sign_ = cfg_.is_positive ? (via_atm ? -1.0f : +1.0f)
                                : (via_atm ? +1.0f : -1.0f);

  // Anti-windup Logic
  float u_mi_req_clamped =  std::clamp(u_mi_req + u_trim_[0], 0.0f, 100.0f);
  float u_ma_req_clamped =  std::clamp(u_ma_req + u_trim_[1], 0.0f, 100.0f);
  float u_at_req_clamped =  std::clamp(u_at_req + u_trim_[2], 0.0f, 100.0f);
  
  bool pos_stop_integration = false;
  bool neg_stop_integration = false;

  if (cfg_.is_positive) {
      if (err > 0.0f) {
          if (u_mi_req_clamped >= 100.0f && u_ma_req_clamped >= 100.0f) {
              pos_stop_integration = true;
          }
      } else {
          if (u_at_req_clamped >= 100.0f) {
              pos_stop_integration = true;
          }
      }
      if (pos_stop_integration) {
        pos_error_integral_ -= err * dt_sec; 
      }
  } else {
      if (err < 0.0f) {
          if (u_mi_req_clamped >= 100.0f && u_ma_req_clamped >= 100.0f) {
              neg_stop_integration = true;
          }
      } else {
          if (u_at_req_clamped >= 100.0f) {
              neg_stop_integration = true;
          }
      }
      if (neg_stop_integration) {
        neg_error_integral_ += err * dt_sec; 
      }
  } 
  
  if (current_time_sec <= 5.0) {
    pos_error_integral_ = 0.0;
    neg_error_integral_ = 0.0;
  }

  return {u_mi_req_clamped, u_ma_req_clamped, u_at_req_clamped};
}

void AcadosMpc::build_mpc_qp(const std::vector<float>& A_seq,
                             const std::vector<Eigen::RowVector3f>& B_seq,
                             float P_now,
                             const std::vector<float>& P_ref,
                             Eigen::MatrixXf& P, Eigen::VectorXf& q,
                             Eigen::MatrixXf& A_con, Eigen::VectorXf& LL, Eigen::VectorXf& UL)
{
  const int NP = cfg_.NP;
  const int nx = cfg_.n_x;
  const int nu = cfg_.n_u;
  const int Nu = nu*NP;
  const int Nx = nx*NP;

  S_bar_.setZero(Nx, Nu);
  T_bar_.setZero(Nx, nx);

  for (int i=0;i<NP;++i) {
    float Ai = 1.f;
    for (int k=0;k<=i;++k) Ai *= A_seq[k];
    T_bar_(i,0) = Ai;

    for (int j=0;j<=i;++j) {
      float A_pow = 1.f;
      for (int k=j+1;k<=i;++k) A_pow *= A_seq[k];
      Eigen::RowVector3f Bl = B_seq[j];
      Eigen::RowVector3f contrib = A_pow * Bl;
      S_bar_.block(i*nx, j*nu, nx, nu) = contrib;
    }
  }

  P = R_ + S_bar_.transpose()*Q_*S_bar_;
  P = 0.5f*(P + P.transpose());

  x0_mpc_(0) = P_now;
  Xref_mpc_ = Eigen::Map<const Eigen::VectorXf>(P_ref.data(), NP);

  qtmp_ = T_bar_ * x0_mpc_ - Xref_mpc_;
  q = S_bar_.transpose() * (Q_ * qtmp_);

  A_con.setIdentity();
  LL = Eigen::VectorXf::Constant(Nu, cfg_.du_min);
  UL = Eigen::VectorXf::Constant(Nu, cfg_.du_max);
}

// Controller.cpp 파일의 solve_qp_first_step 함수 수정

std::array<float,3> AcadosMpc::solve_qp_first_step(const Eigen::MatrixXf& P,
                                                   const Eigen::VectorXf& q,
                                                   const Eigen::MatrixXf& A_con,
                                                   const Eigen::VectorXf& LL,
                                                   const Eigen::VectorXf& UL)
{
  // [수정] 사용하지 않는 파라미터 경고 무시 (Unused parameter warning suppression)
  (void)A_con; 

  if (!qp_) {
      Eigen::VectorXf u = -P.ldlt().solve(q);
      return {u(0), u(1), u(2)};
  }

  // QP Solver (A_con 미사용 — 단순 바운드 구조).
  // hot start 실패는 래퍼가 같은 틱에 cold start 로 복구하므로 여기까지 오지 않는다.
  bool success = qp_->solve(P, q, LL, UL, solution_);

  // 진단: 5000 호출(500 Hz 에서 10 s)마다, 비정상일 때만 보고한다.
  if (++qp_stat_tick_ >= 5000) {
      qp_stat_tick_ = 0;
      const auto st = qp_->take_stats();
      const double hot = st.calls ? 100.0 * (double)st.hot_fail  / (double)st.calls : 0.0;
      const double hard= st.calls ? 100.0 * (double)st.hard_fail / (double)st.calls : 0.0;
      if (hot > 1.0 || st.hard_fail > 0)
          RCLCPP_WARN(rclcpp::get_logger("AcadosMpc"),
            "gid=%d QP: hot-start 실패 %.1f%% (cold 로 복구), 완전 실패 %.2f%% / %ld 호출",
            cfg_.global_id, hot, hard, (long)st.calls);
  }

  if (!success) {
      if (qp_fail_count_++ == 0) {
          RCLCPP_WARN(rclcpp::get_logger("AcadosMpc"),
                      "QP solve failed (gid=%d) — hot/cold 모두 실패, Δu=0.", cfg_.global_id);
      }
      return {0.0f, 0.0f, 0.0f};
  }
  qp_fail_count_ = 0;
  return {solution_(0), solution_(1), solution_(2)};
}

// ============================================================================
// prepare / finish — 중앙집중 MPPI 를 위한 분리
//
// prepare : Bouc-Wen z 갱신 · 피드포워드 역모델 · 적분항 · 크래킹 임계 갱신을 하고
//           uref 를 낸다. **여기까지가 채널별로 독립이다.**
// (그 사이) 채널별 MPPI/QP 또는 **전체 시스템 MPPI** 가 Δu 를 결정한다.
// finish  : 명령 테이퍼 · 클램프 · PWM 스케일 · 밸브 상태 추정 전진.
//
// solve() 는 이 셋을 순서대로 부르는 얇은 껍데기다. 그래서 중앙집중 경로가 기존의
// 튜닝된 피드포워드·적분·크래킹 처리를 **한 줄도 복제하지 않고** 재사용한다.
// ============================================================================
std::array<float,3> AcadosMpc::prepare(float dt_ms, float current_time_sec)
{
  float P_now   = current_P_now_;

  // ── 관측기 보정 (오차·피드포워드보다 **먼저**) ────────────────────────────
  // 직전 틱 끝에서 모델로 한 스텝 예측해 둔 p_hat_ 을 이번 측정으로 보정한다.
  // 비교는 같은 지연 조건에서 해야 하므로 예측값에도 LPF 2단을 복제해 걸고 필터된
  // 측정값과 비교한다. 정상상태에서는 모든 필터가 DC 를 통과시키므로 바이어스가 없다.
  //
  // **일관성이 중요하다.** 추정값을 MPPI 초기 상태에만 쓰고 오차·피드포워드·적분항은
  // 필터값을 쓰면 과도 구간에 둘이 최대 9 kPa 어긋나 MPPI 보정이 피드포워드와 싸운다
  // (계측: 절반 적용 시 압력 RMSE 5.51→4.91 로 개선되지만 위치 IAE 5.30→6.71 로 악화.
  //  일관 적용으로 바꾸자 IAE 5.78, 정착 1.22→1.08, 정상상태 압력오차 0.25→0.00).
  // 안전 트립은 여기서 쓰지 않는다 — 과압 보호는 모델이 아니라 측정에 근거해야 한다.
  if (cfg_.mppi_estimator) {
    if (!p_hat_init_) { p_hat_ = p_hat_f1_ = p_hat_f2_ = P_now; p_hat_init_ = true; }
    const float a = std::clamp(cfg_.obs_bridge_alpha, 0.01f, 1.0f);
    const float b = std::clamp(cfg_.obs_ctrl_alpha,   0.01f, 1.0f);
    p_hat_f1_ = a * p_hat_    + (1.0f - a) * p_hat_f1_;
    p_hat_f2_ = b * p_hat_f1_ + (1.0f - b) * p_hat_f2_;
    const float resid = P_now - p_hat_f2_;
    p_hat_ = std::clamp(p_hat_ + cfg_.obs_gain * resid, 1.0f, 5000.0f);
    obs_resid_acc_ += std::abs((double)resid);
    ++obs_resid_n_;
    P_now = p_hat_;
  } else if (cfg_.mppi_raw_state) {
    P_now = current_P_now_raw_;
  }
  P_used_ = P_now;

  float dt_sec = dt_ms / 1000.0f;
  if (dt_sec <= 0.0001f) dt_sec = cfg_.Ts;
  dt_sec_ = dt_sec;

  uref_ = compute_input_reference(P_now, current_P_micro_, current_P_macro_,
                                  current_P_macro_neg_, dt_sec, current_time_sec);
  return uref_;
}

void AcadosMpc::finish(const std::array<float,3>& du3,
                       std::array<uint16_t, MPC_OUT_DIM>& out3)
{
  // master(위치제어 검증된 버전)와 같이 clamp(uref+du, 0, 100) 를 그대로 낸다.
  // 명령 테이퍼(cmd_taper_kpa)는 롤아웃까지 포함해 완전히 제거했다 — 사람이 정하는
  // kPa 임계값이었고, 플랜트에서만 꺼져 있어 롤아웃과 어긋나 있었다.
  // 비유한 값 차단 — **실기 안전에 필수**. 솔버나 모델이 NaN/Inf 를 내면 uint16 변환이
  // 정의되지 않아 임의의 PWM 이 나갈 수 있다. 그런 값은 0(밸브 닫힘)으로 떨어뜨리고
  // 한 번만 경고한다 (500 Hz 로그 폭주 방지).
  auto safe = [this](float u, const char* what) {
    if (std::isfinite(u)) return u;
    if (nonfinite_cnt_++ == 0)
      RCLCPP_ERROR(rclcpp::get_logger("AcadosMpc"),
        "gid=%d %s 가 비유한 값이다 — 0 으로 대체한다. 모델/솔버를 확인할 것.",
        cfg_.global_id, what);
    return 0.0f;
  };
  std::array<float,3> u0{
    std::clamp(safe(uref_[0] + du3[0], "u_micro"), 0.0f, 100.0f),
    std::clamp(safe(uref_[1] + du3[1], "u_macro"), 0.0f, 100.0f),
    std::clamp(safe(uref_[2] + du3[2], "u_atm"),   0.0f, 100.0f),
  };
  for (auto& v : u0) if (!std::isfinite(v)) v = 0.0f;   // clamp 이후 한 번 더

  // 명령 저역통과 — 밸브 공진과 MPPI 의 틱 단위 스위칭을 끊는다.
  //
  // 이 필터는 선언만 되어 있고 어디서도 호출되지 않았다 (u_lpf_ 사용처 0곳).
  // 그래서 실기에서 지령이 한 틱에 0↔100 으로 튀었고(계측 10000 %/s), 챔버가
  // 8.3 Hz 로 peak-to-peak 96 kPa 진동했다 (실기 20260827_195422).
  // 1차 저역통과는 DC 이득이 1 이라 정상상태 오차를 만들지 않는다 — 느려질 뿐이다.
  //
  // 과압 세이프티는 cmds_ 를 직접 덮어쓰므로 이 필터를 지나지 않는다.
  // **비대칭**이다: 여는 쪽만 느리고 닫는 쪽은 그대로 통과시킨다.
  //
  // 대칭 필터는 닫는 것도 똑같이 늦춘다. 실기에서 밸브가 열리면 챔버가
  // 718 kPa/s 로 차오르는데(20260828 적합), 2 Hz 대칭 필터는 닫는 데 τ=80 ms 를
  // 걸어 그동안 ≈57 kPa 를 더 밀어 넣는다 — 진동을 막으려던 필터가 오버슛을
  // 만든다. 여는 쪽만 늦추면 공진을 때리는 급격한 상승은 그대로 막으면서
  // 배기·폐쇄는 즉시 듣는다. 과압 세이프티는 어차피 cmds_ 를 직접 덮어쓴다.
  if (cfg_.cmd_lpf_hz > 0.0f && dt_sec_ > 1e-6f) {
    if (!u_lpf_init_) { u_lpf_ = u0; u_lpf_init_ = true; }
    const float a = 1.0f - std::exp(-2.0f * (float)M_PI * cfg_.cmd_lpf_hz * (float)dt_sec_);
    for (int j = 0; j < 3; ++j) {
      if (u0[(size_t)j] <= u_lpf_[(size_t)j]) u_lpf_[(size_t)j] = u0[(size_t)j];  // 닫기: 즉시
      else u_lpf_[(size_t)j] += a * (u0[(size_t)j] - u_lpf_[(size_t)j]);          // 열기: 완만히
      u0[(size_t)j] = std::clamp(u_lpf_[(size_t)j], 0.0f, 100.0f);
    }
  }

  // 크래킹 하한은 **모든 것의 맨 마지막**이다 — LPF 뒤여야 한다.
  //
  // 위의 LPF 는 비대칭이라 "닫을 땐 즉시" 경로가 있다. 지령이 한 틱이라도
  // 내려가면 필터가 그 값으로 곧장 떨어지므로, 출력은 평균이 아니라 최근
  // **최솟값**에 눌러앉는다. 하한을 LPF 앞에 걸면 그 최솟값이 하한을 밑돌아
  // 무효가 된다 (시뮬: 하한 55.8% 인데 최종 지령 48.5%, 오차 +0.95 kPa 고착).
  // 유량을 요구한 밸브는 **최소한 크래킹까지는 연다.**
  //
  // 역모델은 요구 유량이 작으면 작은 지령을 낸다. 그런데 이 밸브는 크래킹
  // 아래에서 유량이 정확히 0 이라, 그 지령은 "조금 흐른다" 가 아니라 "아무것도
  // 안 흐른다" 가 된다. 피드백이 없으니 오차가 그대로 얼어붙는다.
  //   실기 20260828_153540: 하강 목표에서 배기가 122~133 mA 에 앉아 오차
  //   +3.4 kPa 가 6 초 동안 그대로였다. 실측 배기 크래킹은 138 mA 다.
  //
  // 이것도 적분 보정과 같은 이유로 MPPI **뒤**여야 한다. 앞에 걸면 MPPI 가
  // 같은 모델로 "그만큼 열면 넘친다" 고 판단해 du 로 도로 끌어내린다.
  //   시뮬에서 피드포워드에만 걸었을 때: 크래킹 하한 54.3% 인데 최종 지령이
  //   45.9% 였고 오차가 +1.05 kPa 에 정확히 얼어붙었다.
  //
  // 최소 유량은 A_max 의 valve_crack_area_frac(0.05) 만큼이다 — 충전 ≈72 kPa/s,
  // 배기 ≈9 kPa/s 로 과하지 않다. u_crack_ 은 예전부터 계산만 되고 아무도
  // 쓰지 않았다 (접근자 u_crack() 도 호출처가 없었다).
  // **압력이 실제로 안 움직일 때만** 건다. 그것이 이 하한의 원래 목적이다 —
  // "유량을 요구했는데 아무 일도 안 일어난다" 를 구제하는 것.
  //
  // 조건 없이 걸면 반대 문제가 생긴다. 모델은 크래킹 지점의 최소 유량을
  // 14.5 kPa/s 로 보지만 실기 충전 밸브는 그 지점에서 285~495 kPa/s 를 낸다
  // (20260828_160825, 지령 41%/104 mA). 압력이 목표보다 조금만 내려가도 하한이
  // 밸브를 확 열어 40 ms 만에 +6 kPa 를 밀어 넣고, 배기가 다시 끌어내리는
  // 3 Hz 리밋사이클이 된다 (p-p 14~30 kPa).
  // 압력이 이미 움직이고 있으면 피드백이 살아 있다는 뜻이므로 하한이 필요 없다.
  // 조건은 "압력이 **원하는 방향으로** 충분히 움직이는가" 다. 절댓값으로 보면
  // 안 된다 — 압력이 반대로 흐르는 중일 때(정확히 하한이 가장 필요한 때) 게이트가
  // 오히려 하한을 풀어 버린다.
  //   실기 20260828_163335: t=62.0~63.5 에 배기를 크래킹(54.9%/137 mA)에 둔 채
  //   114.3 kPa 를 ±0.4 로 1.5 초간 완벽히 유지하고 있었다. 그런데 압력이
  //   +8 kPa/s 로 **올라가기 시작하자** |dP/dt|>5 라는 이유로 하한이 풀렸고,
  //   배기 지령이 11.6% 로 떨어져 아무것도 못 하는 사이 121.8 kPa 까지 올랐다.
  //   그때부터 2 Hz · p-p 40 kPa 진동이 시작됐다.
  // 오차가 아주 작을 때는 걸지 않는다.
  //
  // 하한은 "요구했는데 아무 일도 안 일어난다" 를 구제하려는 것이지, 0.1 kPa
  // 를 다듬으려는 것이 아니다. 크래킹 지점의 한 펄스가 수 kPa 를 움직이므로,
  // 그보다 작은 오차에 하한을 걸면 매번 목표를 넘어가 방향이 뒤집히고 릴레이
  // 진동이 된다.
  //   실기 20260828_174653: t=39.8~41.5 에 n1 을 43.0% 에 두고 오차 +0.1 로
  //   1.7 초간 완벽히 안정했다. 그런데 오차가 −0.1 로 넘어가는 순간 반대 밸브
  //   (n2)가 하한으로 발사돼 +8.5 kPa 를 밀어 올렸고 2.8 Hz 진동이 재개됐다.
  // 이 문턱 아래에서는 적분 트림(하한 뒤에 더해진다)이 계속 다듬으므로
  // 정상상태 오차가 남지는 않는다.
  if (err_abs_ >= cfg_.crack_floor_min_err_kpa &&
      want_sign_ * dpdt_f_ < cfg_.crack_floor_rate_kpas) {
    for (int j = 0; j < 3; ++j) {
      if (u_want_[(size_t)j] && u0[(size_t)j] < u_crack_[(size_t)j])
        u0[(size_t)j] = std::clamp(u_crack_[(size_t)j], 0.0f, 100.0f);
    }
  }

  // 적분 보정은 **맨 마지막**이다 — MPPI·LPF·크래킹 하한 전부 뒤.
  //
  // uref 안에 넣으면 MPPI 가 그대로 되돌린다. MPPI 는 같은 밸브 모델로 재최적화
  // 하는데, 적분항이 모델이 요구하는 것보다 밸브를 더 여는 순간 MPPI 의 예측은
  // "그러면 넘친다" 가 되어 du 로 그만큼 빼 버린다. 적분항이 메우려는 것이 바로
  // 그 모델의 오차이므로, 같은 모델에게 심판을 맡기면 영원히 못 메운다.
  //   실기 20260828_151518: 배기 피드포워드 36.1% + 적분 +10 = 46.1% 여야 하는데
  //   실제 지령은 38.1% 에 고정됐고(MPPI 가 −8), 챔버가 11 초간 목표보다
  //   16.4 kPa 높은 채로 멈춰 있었다. 실측 배기 크래킹은 38.1~45% 사이다.
  //
  // 크래킹 하한 **앞**에 두어도 같은 이유로 무력해진다. MPPI 출력이 0 이면
  // 트림(최대 ki_u_limit_pct = 15%p)만으로는 하한(예: 57.4%)을 넘지 못하고,
  // 하한이 그것을 덮어써 지령이 하한값에 그대로 얼어붙는다.
  //   실기 20260828_173347: ref 101.5 인데 챔버가 103.1 에서 4 초간 멈췄고
  //   배기 지령이 57.4%(= 크래킹 하한)에 고정돼 있었다. 사용자가 말한
  //   "시간이 지나도 안 사라지는 3~4 kPa 정상상태 오차" 가 이것이다.
  for (int j = 0; j < 3; ++j) {
    if (u0[(size_t)j] > 0.0f || u_trim_[(size_t)j] > 0.0f)   // 닫으라는 밸브는 건드리지 않는다
      u0[(size_t)j] = std::clamp(u0[(size_t)j] + u_trim_[(size_t)j], 0.0f, 100.0f);
  }



  last_u3_ = u0;



  // 밸브 내부 상태(q, qd) 추정을 실제 인가 명령 + 측정 압력으로 한 틱 전진시킨다.
  // 관측기를 쓰면 압력 예측도 이어 간다 (매 틱 측정으로 리셋하지 않는다).
  mppi::Exogenous ex;
  ex.P_micro = current_P_micro_;
  ex.P_macro = current_P_macro_;
  ex.P_atm   = current_P_atm_;
  ex.V0      = cfg_.volume_m3;
  ex.Vdot    = vol_dot_est_;
  ex.P_ref   = cfg_.ref_value;
  plant_est_.P = (cfg_.mppi_estimator && p_hat_init_) ? p_hat_ : P_used_;
  plant_est_.v[0].z = (float)z_micro_;  plant_est_.v[0].dir = dir_micro_;
  plant_est_.v[1].z = (float)z_macro_;  plant_est_.v[1].dir = dir_macro_;
  plant_est_.v[2].z = (float)z_atm_;    plant_est_.v[2].dir = dir_atm_;
  mppi::advance_valve_estimate(mppi_pv_, plant_est_, u0, ex, dt_sec_,
                               cfg_.mppi_estimator, cfg_.volume_m3);

  // 적응 ①의 기준 — **정적 유량(q_static)이 아니라 동적 유량**을 써야 한다.
  // 밸브는 2차 동특성(wn≈40 rad/s, τ≈25 ms)을 지나므로 과도 구간에서 실제 유량이
  // 정적값보다 한참 작다. 정적값과 비교하면 비가 늘 1 보다 작게 나와, 시뮬처럼
  // 모델과 플랜트가 같은 경우에도 k 가 0.33 까지 내려갔다.
  // advance_valve_estimate 이후의 plant_est_.v[j].q 가 mppi::step 이 챔버에
  // 적용하는 바로 그 값이고, 부호 규약도 거기와 같게 맞춘다.
  {
    const float qmi = plant_est_.v[mppi::V_MICRO].q;
    const float qma = plant_est_.v[mppi::V_MACRO].q;
    const float qat = plant_est_.v[mppi::V_ATM].q;
    q_model_last_ = cfg_.is_positive ? (qmi + qma - qat) : (-qmi - qma + qat);
  }
  if (cfg_.mppi_estimator) p_hat_ = plant_est_.P;

  // 4095 스케일 (100% -> 4095), 출력 순서는 micro, atm, macro
  out3[0] = static_cast<uint16_t>( std::round(u0[0] * 40.95f) );
  out3[1] = static_cast<uint16_t>( std::round(u0[2] * 40.95f) );
  out3[2] = static_cast<uint16_t>( std::round(u0[1] * 40.95f) );
}

mppi::ChannelState AcadosMpc::rollout_state() const
{
  mppi::ChannelState x0;
  x0.P = P_used_;
  const double zs[3]  = { z_micro_,      z_macro_,      z_atm_      };
  const double pis[3] = { prev_I_micro_, prev_I_macro_, prev_I_atm_ };
  const int    dis[3] = { dir_micro_,    dir_macro_,    dir_atm_    };
  for (int j = 0; j < 3; ++j) {
    x0.v[(size_t)j].q     = plant_est_.v[(size_t)j].q;
    x0.v[(size_t)j].qd    = plant_est_.v[(size_t)j].qd;
    x0.v[(size_t)j].z     = (float)zs[j];
    x0.v[(size_t)j].prevI = (float)pis[j];
    x0.v[(size_t)j].dir   = dis[j];
  }
  return x0;
}

void AcadosMpc::solve(float dt_ms,
                      std::array<uint16_t, MPC_OUT_DIM>& out3,
                      float current_time_sec)
{
  // 채널 독립 경로: prepare → (MPPI 또는 QP) → finish.
  // 중앙집중 경로(Controller::run_system_mppi)는 prepare 와 finish 사이에
  // 전체 시스템 솔버를 끼워 넣는다 — 같은 피드포워드·적분·크래킹 처리를 공유한다.
  const auto uref_arr = prepare(dt_ms, current_time_sec);
  Eigen::RowVector3f u_ref(uref_arr[0], uref_arr[1], uref_arr[2]);

  std::fill(P_ref_.begin(), P_ref_.end(), ref_eff_);

  mppi::Exogenous ex;
  ex.P_micro = current_P_micro_;
  ex.P_macro = current_P_macro_;
  ex.P_atm   = current_P_atm_;
  ex.V0      = cfg_.volume_m3;
  ex.Vdot    = vol_dot_est_;
  ex.P_ref   = ref_eff_;

  // ── 보강 ③ 접근 시상수 자동 조정 ───────────────────────────────────
  // 오차 부호가 자주 바뀌면(진동) 느리게, 한 방향으로 크게 남으면(둔함) 빠르게 민다.
  // 경계 안에서만 움직이고, 끄면 즉시 설정값으로 되돌아간다.
  const float tau_cfg = (cfg_.mppi_ref_tau_s > 0.f) ? cfg_.mppi_ref_tau_s
                                                    : cfg_.target_time_constant;
  if (cfg_.aug.auto_tune) {
    if (tau_ref_cur_ <= 0.0f) tau_ref_cur_ = tau_cfg;
    const float e = ref_eff_ - P_used_;
    const int sg = (e > 0.05f) ? 1 : (e < -0.05f ? -1 : 0);
    if (sg != 0) {
      if (last_err_sign_ != 0 && sg != last_err_sign_) ++sign_flips_;
      last_err_sign_ = sg;
    }
    err_abs_acc_ += std::abs(e);
    if (++tune_tick_ >= cfg_.aug.tune_window) {
      const float osc = (float)sign_flips_ / (float)tune_tick_;
      const float err_mean = err_abs_acc_ / (float)tune_tick_;
      if (osc > cfg_.aug.osc_hi)
        tau_ref_cur_ *= (1.0f + cfg_.aug.tune_rate);          // 진동 → 느리게
      else if (err_mean > cfg_.aug.err_slow_kpa)
        tau_ref_cur_ *= (1.0f - cfg_.aug.tune_rate);          // 둔함 → 빠르게
      tau_ref_cur_ = std::clamp(tau_ref_cur_, cfg_.aug.tau_min, cfg_.aug.tau_max);
      tune_tick_ = 0; sign_flips_ = 0; err_abs_acc_ = 0.0f;
    }
  } else {
    tau_ref_cur_ = tau_cfg;
  }
  ex.tau_ref = tau_ref_cur_;
  ex.P0      = P_used_;

  std::array<float,3> du3{0.f, 0.f, 0.f};

  if (mppi_) {
    du3 = mppi_->solve(rollout_state(), ex, uref_arr);
    report_mppi_stats();
  } else {
    update_linearization(cfg_.ref_value, u_ref);
    const int Nu = cfg_.n_u * cfg_.NP;
    Pmat_.setZero(Nu, Nu); qvec_.setZero(Nu); Acon_.setZero(Nu, Nu);
    LL_.setZero(Nu); UL_.setZero(Nu);
    build_mpc_qp(A_seq_, B_seq_, P_used_, P_ref_, Pmat_, qvec_, Acon_, LL_, UL_);
    for (int i = 0; i < cfg_.NP; ++i)
      for (int j = 0; j < 3; ++j) {
        const int idx = i * 3 + j;
        LL_(idx) = std::max(-uref_arr[(size_t)j], cfg_.du_min);
        UL_(idx) = std::min(100.0f - uref_arr[(size_t)j], cfg_.du_max);
      }
    du3 = solve_qp_first_step(Pmat_, qvec_, Acon_, LL_, UL_);
  }

  finish(du3, out3);
}

// 진단 로그 — 채널 경로와 중앙집중 경로가 공유한다.
void AcadosMpc::report_mppi_stats()
{
  if (!mppi_) return;
  if (++mppi_stat_tick_ < 5000) return;
  mppi_stat_tick_ = 0;
  const auto st = mppi_->take_stats();
  if (!st.calls) return;
  // 유효샘플·이상치배율은 **활성 틱** 에서만 누적된다. 전체 틱으로 나누면 평평 틱
  // 비율만큼 축소돼 보여 진단을 오독하게 된다 (평평 74% 이면 실제값의 26%).
  const double active = std::max<double>(1.0, (double)(st.calls - st.flat));
  RCLCPP_INFO(rclcpp::get_logger("Mppi"),
    "gid=%d MPPI: %.0f us 평균 / %.0f us 최대 (틱 예산 대비), "
    "유효샘플 %.1f/%d, Jmin %.4f, 초과 %.4f, 이상치배율 %.1f, "
    "첫스텝 포화 %.1f%%, 평평 %.1f%%",
    cfg_.global_id, st.sum_us / (double)st.calls, (double)st.max_us,
    st.sum_eff / active, mppi_->params().K, st.sum_cost / (double)st.calls,
    st.sum_spread / (double)st.calls, st.sum_outlier / active,
    100.0 * (double)st.sat_first / (double)st.calls,
    100.0 * (double)st.flat / (double)st.calls);
  if (cfg_.mppi_estimator && obs_resid_n_ > 0) {
    RCLCPP_INFO(rclcpp::get_logger("Mppi"),
      "gid=%d 관측기: 평균 |잔차| %.3f kPa (이득 %.2f)",
      cfg_.global_id, obs_resid_acc_ / (double)obs_resid_n_, (double)cfg_.obs_gain);
    obs_resid_acc_ = 0.0; obs_resid_n_ = 0;
  }
}


// ================================
// Controller
// ================================
Controller::Controller(const rclcpp::NodeOptions& opts)
: rclcpp::Node("pp_controller", opts)
{
  period_ms_ = this->declare_parameter<int>("period_ms", 2);  // 2ms default (500Hz event-driven)
  enable_thread_pinning_ = this->declare_parameter<bool>("enable_thread_pinning", true);
  cpu_pins_param_ = this->declare_parameter<std::vector<int64_t>>("cpu_pins", std::vector<int64_t>{0,1,2,3});

  num_positive_channels_ = this->declare_parameter<int>("num_positive_channels", 8);
  num_total_channels_   = this->declare_parameter<int>("num_total_channels", 12);
  num_actuators_        = this->declare_parameter<int>("num_actuators", 1);

  // 채널 수에 맞게 동적 벡터 초기화
  channel_configs_.resize(num_total_channels_);
  mpc_ref_kpa_.assign(num_total_channels_, 101.325);
  ref_snapshot_.assign(num_total_channels_, 0.0);
  final_active_vols_ml_.assign(num_total_channels_, 0.0);

  sensor_.atm_offset = get_param_or<double>(this, "Sensor_calibration.atm_offset", 101.325);
  zero_tolerance_kpa_ = get_param_or<double>(this, "Sensor_calibration.zero_tolerance_kpa", 8.0);
  encoder_zero_when_disconnected_ =
      get_param_or<bool>(this, "encoder_zero_when_disconnected", true);
  use_measured_dt_ = get_param_or<bool>(this, "use_measured_dt", true);

  sensor_filter_alpha_ = this->declare_parameter<double>("sensor_filter_alpha", 1.0);

  if (sensor_filter_alpha_ <= 0.0) sensor_filter_alpha_ = 0.01;
  if (sensor_filter_alpha_ > 1.0)  sensor_filter_alpha_ = 1.0;

  RCLCPP_INFO(get_logger(), "Sensor Filter Alpha applied: %.3f", sensor_filter_alpha_);

  // Flat per-board calibration: Sensor_calibration.boards."N".offset/gain
  for (int bid = 1; bid <= NUM_CAN_BOARDS; ++bid) {
    const std::string base = "Sensor_calibration.boards." + std::to_string(bid);
    auto& ch = sensor_.boards[(size_t)(bid - 1)];
    ch.offset = get_param_or<double>(this, base + ".offset", ch.offset);
    if (bid - 1 >= 0 && bid - 1 < NUM_CAN_BOARDS)
      yaml_offset_[(size_t)(bid - 1)] = ch.offset;   // 0점 재보정 이탈 경고 기준
    ch.gain   = get_param_or<double>(this, base + ".gain",   ch.gain);
  }

  channel_board_offset_ = get_param_or<int>(this, "channel_board_offset", 4);
  P_pos_board_id_       = get_param_or<int>(this, "line_pressure_boards.pos",       1);
  P_neg_board_id_       = get_param_or<int>(this, "line_pressure_boards.neg",       2);
  P_macro_board_id_     = get_param_or<int>(this, "line_pressure_boards.macro",     3);
  P_macro_neg_board_id_ = get_param_or<int>(this, "line_pressure_boards.macro_neg", 4);

  RCLCPP_INFO(this->get_logger(), "================ PARAMETER DIAGNOSIS ================");
  RCLCPP_INFO(this->get_logger(), "Loaded parameter [Sensor_calibration.boards.4.offset]: %f", sensor_.boards[3].offset);
  RCLCPP_INFO(this->get_logger(), "channel_board_offset=%d  P_pos_board=%d  P_neg_board=%d  P_macro_board=%d",
              channel_board_offset_, P_pos_board_id_, P_neg_board_id_, P_macro_board_id_);
  RCLCPP_INFO(this->get_logger(), "=====================================================");


  mpc_.NP             = get_param_or<int>(this,    "MPC_parameters.NP", 5);
  mpc_.n_x            = get_param_or<int>(this,    "MPC_parameters.n_x", 1);
  mpc_.n_u            = get_param_or<int>(this,    "MPC_parameters.n_u", 3);
  mpc_.Ts             = get_param_or<double>(this, "MPC_parameters.Ts",  0.01);
  mpc_.Q_value        = get_param_or<double>(this, "MPC_parameters.Q_values", 10.0);
  mpc_.R_value        = get_param_or<double>(this, "MPC_parameters.R_values",  1.0);
  mpc_.ejector_k       = get_param_or<double>(this, "MPC_parameters.ejector_k", 0.005);
  mpc_.ejector_p_limit = get_param_or<double>(this, "MPC_parameters.ejector_p_limit", 11.325);
  mpc_.leakage_u_pos = get_param_or<double>(this, "MPC_parameters.leakage_u_pos", 0.0);
  mpc_.leakage_u_neg = get_param_or<double>(this, "MPC_parameters.leakage_u_neg", 0.0);
  mpc_.target_tc = get_param_or<double>(this, "MPC_parameters.target_time_constant", 0.2);
  mpc_.valve_crack_area_frac = get_param_or<double>(this, "MPC_parameters.valve_crack_area_frac", 1e-6);
  mpc_.cmd_lpf_hz            = get_param_or<double>(this, "MPC_parameters.cmd_lpf_hz", 0.0);
  mpc_.ki_u_limit_pct        = get_param_or<double>(this, "MPC_parameters.ki_u_limit_pct", 10.0);
  mpc_.ki_flow               = get_param_or<double>(this, "MPC_parameters.ki_flow", 0.02);
  mpc_.q_trim_limit          = get_param_or<double>(this, "MPC_parameters.q_trim_limit", 2.0);
  mpc_.crack_floor_rate_kpas = get_param_or<double>(this, "MPC_parameters.crack_floor_rate_kpas", 5.0);
  mpc_.crack_floor_min_err_kpa = get_param_or<double>(this, "MPC_parameters.crack_floor_min_err_kpa", 1.5);
  mpc_.integ_hold_rate_kpas    = get_param_or<double>(this, "MPC_parameters.integ_hold_rate_kpas", 0.0);
  mpc_.integ_deadzone_boost    = get_param_or<double>(this, "MPC_parameters.integ_deadzone_boost", 1.0);

  // ── 솔버 선택 ────────────────────────────────────────────────────────────
  mpc_.solver = get_param_or<std::string>(this, "MPC_parameters.solver", std::string("qp"));
  mpc_.mppi_samples    = get_param_or<int>   (this, "MPC_parameters.mppi_samples",    128);
  mpc_.mppi_lambda     = get_param_or<double>(this, "MPC_parameters.mppi_lambda",     0.30);
  mpc_.mppi_sigma_pct  = get_param_or<double>(this, "MPC_parameters.mppi_sigma_pct",  8.0);
  mpc_.mppi_sigma_explore_pct = get_param_or<double>(this, "MPC_parameters.mppi_sigma_explore_pct", 30.0);
  mpc_.mppi_explore_frac      = get_param_or<double>(this, "MPC_parameters.mppi_explore_frac",       0.30);
  mpc_.mppi_du_limit_pct      = get_param_or<double>(this, "MPC_parameters.mppi_du_limit_pct",     100.0);
  mpc_.mppi_ref_tau_s         = get_param_or<double>(this, "MPC_parameters.mppi_ref_tau_s",         -1.0);
  mpc_.mppi_np                = get_param_or<int>   (this, "MPC_parameters.mppi_np",                  -1);
  mpc_.mppi_ts_s              = get_param_or<double>(this, "MPC_parameters.mppi_ts_s",              -1.0);
  mpc_.mppi_noise_beta = get_param_or<double>(this, "MPC_parameters.mppi_noise_beta", 0.70);
  mpc_.mppi_w_track    = get_param_or<double>(this, "MPC_parameters.mppi_w_track",   -1.0);
  mpc_.mppi_w_effort   = get_param_or<double>(this, "MPC_parameters.mppi_w_effort",  -1.0);
  mpc_.mppi_w_du       = get_param_or<double>(this, "MPC_parameters.mppi_w_du",       0.05);
  mpc_.mppi_track_scale_kpa = get_param_or<double>(this, "MPC_parameters.mppi_track_scale_kpa", 10.0);
  mpc_.mppi_terminal_mult   = get_param_or<double>(this, "MPC_parameters.mppi_terminal_mult",    5.0);
  mpc_.mppi_substeps        = get_param_or<int>   (this, "MPC_parameters.mppi_substeps",           2);
  mpc_.mppi_raw_state       = get_param_or<bool>  (this, "MPC_parameters.mppi_raw_state",       false);
  rail_rate_enable_         = get_param_or<bool>  (this, "MPC_parameters.mppi_rail_rate",      false);
  mpc_.mppi_estimator       = get_param_or<bool>  (this, "MPC_parameters.mppi_estimator",       false);
  mpc_.obs_gain             = get_param_or<double>(this, "MPC_parameters.obs_gain",              0.10);
  mpc_.obs_bridge_alpha     = get_param_or<double>(this, "MPC_parameters.obs_bridge_alpha",      0.2);
  {
    std::string sv = mpc_.solver;
    std::transform(sv.begin(), sv.end(), sv.begin(), ::tolower);
    if (sv != "qp" && sv != "mppi" && sv != "mppi_system") {
      RCLCPP_WARN(get_logger(),
        "MPC_parameters.solver='%s' 는 알 수 없다 — 'qp' 로 진행한다 "
        "(선택: qp | mppi | mppi_system)", mpc_.solver.c_str());
      sv = "qp";
    }
    mpc_.solver = sv;
  }


  default_volume_ml_  = get_param_or<double>(this, "default_volume_ml",    1.0);
  actuator_connected_ = get_param_or<bool>  (this, "actuator_connected",   true);
  tank_volume_pos_ml_ = get_param_or<double>(this, "tank_volume_pos_ml", 750.0);
  tank_volume_neg_ml_ = get_param_or<double>(this, "tank_volume_neg_ml", 400.0);

  vol_ml_.resize(num_total_channels_);
  vol_scale_.assign(num_total_channels_, 1.0);
  for (int i = 0; i < num_total_channels_; ++i) {
    if (!actuator_connected_)
      vol_ml_[i] = (i < num_positive_channels_) ? tank_volume_pos_ml_ : tank_volume_neg_ml_;
    else
      vol_ml_[i] = default_volume_ml_;

    // 채널별 부피 오버라이드 — channel_config.chN.volume_ml
    //
    // 챔버 부피는 요구 유량에 그대로 곱해진다 (m_dot = P_dot·V/(R·T)). 전 채널을
    // 같은 값으로 두면 실제 부피가 큰 채널은 필요한 유량의 일부만 요구해 느리고,
    // 작은 채널은 과요구해 진동한다.
    //   실기 20260829_142216 (6축, 공급 정상): 같은 전류·차압에서 dP/dt 를 비교하면
    //   유효 부피가 채널 간 수 배 차이났다. ch1·ch2 가 크고(정착 8~22 초, 밸브
    //   통전 8~12%), ch3·ch4 가 작았다(ch4 는 p-p 33~45 kPa 로 진동).
    //   크기의 절대값은 그 로그로 확정하지 못했지만(창을 좁히면 추정이 흩어진다)
    //   순서는 일관됐다. 채널을 하나씩 돌려 재면 확정된다.
    const double v_ov = get_param_or<double>(
        this, "channel_config.ch" + std::to_string(i) + ".volume_ml", 0.0);
    if (v_ov > 0.0) vol_ml_[i] = v_ov;

    // 배율 — 액추에이터 연결 시 기하 모델 값에 곱한다 (volume_ml 은 그때 덮어써진다).
    vol_scale_[(size_t)i] = std::max(1e-3, get_param_or<double>(
        this, "channel_config.ch" + std::to_string(i) + ".volume_scale", 1.0));
  }
  {
    std::string vs;
    for (int i = 0; i < num_total_channels_; ++i) {
      char b[48];
      if (std::abs(vol_scale_[(size_t)i] - 1.0) > 1e-6)
        snprintf(b, sizeof(b), " ch%d=%.0f(x%.2f)", i, vol_ml_[(size_t)i], vol_scale_[(size_t)i]);
      else snprintf(b, sizeof(b), " ch%d=%.0f", i, vol_ml_[(size_t)i]);
      vs += b;
    }
    RCLCPP_INFO(get_logger(), "[채널 부피 mL]%s", vs.c_str());
  }
  prev_vol_m3_.resize(num_total_channels_);
  for (int i = 0; i < num_total_channels_; ++i)
    prev_vol_m3_[i] = vol_ml_[i] * 1.0e-6;

  RCLCPP_INFO(get_logger(), "Actuator: %s | vol_pos=%.0f mL, vol_neg=%.0f mL",
              actuator_connected_ ? "CONNECTED" : "DISCONNECTED",
              tank_volume_pos_ml_, tank_volume_neg_ml_);

  for(int i = 0; i < num_total_channels_; ++i) {
    std::string prefix = "channel_config.ch" + std::to_string(i) + ".";
    
    channel_configs_[i].pos_ki_micro = get_param_or<double>(this, prefix + "pos_ki_micro", 0.0);
    channel_configs_[i].pos_ki_macro = get_param_or<double>(this, prefix + "pos_ki_macro", 0.0);
    channel_configs_[i].pos_ki_atm   = get_param_or<double>(this, prefix + "pos_ki_atm",   0.0);
    
    channel_configs_[i].neg_ki_micro = get_param_or<double>(this, prefix + "neg_ki_micro", 0.0);
    channel_configs_[i].neg_ki_macro = get_param_or<double>(this, prefix + "neg_ki_macro", 0.0);
    channel_configs_[i].neg_ki_atm   = get_param_or<double>(this, prefix + "neg_ki_atm",   0.0);

    // ── 밸브별 13-parameter ─────────────────────────────────────────────────
    // 우선순위: chN.<role>.<param>  (valve_fit_solve.py 가 쓰는 형태)
    //        → chN.<param>          (예전 평면 형태 = 세 밸브 공용)
    //        → 하드코딩 기본값
    // 이렇게 하면 피팅 파일을 병합하는 순간 밸브별 값이 실제로 쓰이고, 없으면
    // 기존 동작과 완전히 같다.
    {
      auto flat = [&](const char* n, double dflt) {
        return get_param_or<double>(this, prefix + n, dflt);
      };
      // 평면 값(또는 기본값)을 먼저 읽어 세 밸브의 폴백으로 쓴다.
      Valve13 base;
      base.I_MAX       = flat("I_MAX",       0.30);
      base.A_max       = flat("A_max",       0.2845);
      base.k_shape     = flat("k_shape",     33.09);
      base.C_k         = flat("C_k",         0.0288);
      base.C_p         = flat("C_p",         0.00012);
      base.C_z         = flat("C_z",         0.0);
      base.A_bw        = flat("A_bw",        260649.5);
      base.beta_bw     = flat("beta_bw",     179.0);
      base.gamma_bw    = flat("gamma_bw",    0.06);
      base.alpha_shape = flat("alpha_shape", 3884.2);
      base.wn_up       = flat("wn_up",       40.0);
      base.zeta_up     = flat("zeta_up",     1.2);
      base.wn_down     = flat("wn_down",     45.0);
      base.zeta_down   = flat("zeta_down",   1.0);

      // mppi::ValveIdx 와 같은 순서: 0=micro, 1=macro, 2=atm
      static const char* kRole[3] = {"micro", "macro", "atm"};
      bool any_per_valve = false;
      for (int j = 0; j < 3; ++j) {
        const std::string rp = prefix + kRole[j] + ".";
        auto pv = [&](const char* n, double dflt) {
          const double v = get_param_or<double>(this, rp + n, dflt);
          if (v != dflt) any_per_valve = true;   // 피팅 파일에서 실제로 읽혔다는 신호
          return v;
        };
        auto& t = channel_configs_[i].v[(size_t)j];
        t.I_MAX       = pv("I_MAX",       base.I_MAX);
        t.A_max       = pv("A_max",       base.A_max);
        t.k_shape     = pv("k_shape",     base.k_shape);
        t.C_k         = pv("C_k",         base.C_k);
        t.C_p         = pv("C_p",         base.C_p);
        t.C_z         = pv("C_z",         base.C_z);
        t.A_bw        = pv("A_bw",        base.A_bw);
        t.beta_bw     = pv("beta_bw",     base.beta_bw);
        t.gamma_bw    = pv("gamma_bw",    base.gamma_bw);
        t.alpha_shape = pv("alpha_shape", base.alpha_shape);
        t.wn_up       = pv("wn_up",       base.wn_up);
        t.zeta_up     = pv("zeta_up",     base.zeta_up);
        t.wn_down     = pv("wn_down",     base.wn_down);
        t.zeta_down   = pv("zeta_down",   base.zeta_down);
      }
      channel_configs_[i].per_valve_loaded = any_per_valve;
      channel_configs_[i].chamber_volume_ml =
          get_param_or<double>(this, prefix + "chamber_volume_ml", -1.0);
    }
  }

  sys_valve_operate_   = get_param_or<bool>(this, "system_parameters.valve_operate",   false);

  macro_switch_pwm_index_     = get_param_or<int>(this, "MacroSwitch.pwm_index", 9);

  pid_pos_.kp  = get_param_or<double>(this, "LinePID.pos.kp",  0.5);
  pid_pos_.ki  = get_param_or<double>(this, "LinePID.pos.ki",  0.0);
  pid_pos_.kd  = get_param_or<double>(this, "LinePID.pos.kd",  0.0);
  pid_pos_.ref = get_param_or<double>(this, "LinePID.pos.ref", 150.0);
  pid_out_min_ = get_param_or<double>(this, "LinePID.out_min", 0.0);
  pid_out_max_ = get_param_or<double>(this, "LinePID.out_max", 100.0);
  // flat index: (P_pos_board_id-1)*3 + 0 = (1-1)*3+0 = 0
  pid_pos_pwm_index_ = get_param_or<int>(this, "LinePID.pos.pwm_index", 0);

  pid_neg_.kp  = get_param_or<double>(this, "LinePID.neg.kp",  0.5);
  pid_neg_.ki  = get_param_or<double>(this, "LinePID.neg.ki",  0.0);
  pid_neg_.kd  = get_param_or<double>(this, "LinePID.neg.kd",  0.0);
  pid_neg_.ref = get_param_or<double>(this, "LinePID.neg.ref", 20.0);
  // flat index: (P_neg_board_id-1)*3 + 0 = (2-1)*3+0 = 3
  pid_neg_pwm_index_ = get_param_or<int>(this, "LinePID.neg.pwm_index", 3);

  ref_client_cfg_.enable        = get_param_or<bool>(this,  "RefTcp.enable",        false);
  ref_client_cfg_.host = get_param_or<std::string>(this, "RefTcp.host", "169.254.46.254");
  ref_client_cfg_.port          = get_param_or<int>(this,   "RefTcp.port",          2292);
  
  ref_client_cfg_.expect_n       = get_param_or<int>(this,   "RefTcp.expect_n",       num_total_channels_);
  ref_client_cfg_.pressure_scale = get_param_or<double>(this,"RefTcp.pressure_scale", 1.0 / 327.675);
  
  log_channel_id_ = this->declare_parameter<int>("log_channel_id", -1);

  if (ref_client_cfg_.enable) {
    ref_client_ = std::make_unique<RefTcpClient>(
      ref_client_cfg_,
      [this](const std::vector<double>& arr){
        if ((int)arr.size() < num_total_channels_) return;

        std::lock_guard<std::mutex> lk(mpc_ref_mtx_);

        for(int i = 0; i < num_total_channels_; ++i) {
            mpc_ref_kpa_[i] = arr[i];
        }
      }
    );
}

  std::fill(mpc_ref_kpa_.begin(), mpc_ref_kpa_.end(), 101.325);

  auto reliable = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)).reliable().keep_last(5);
  sub_sensors_ = create_subscription<std_msgs::msg::UInt16MultiArray>(
      "board/sensors", reliable, std::bind(&Controller::on_sensor, this, _1));

  pub_pwm_cmd_   = create_publisher<std_msgs::msg::UInt16MultiArray>("board/pwm_cmd", 5);
  pub_mpc_refs_  = create_publisher<std_msgs::msg::Float64MultiArray>("controller/mpc_refs_kpa", 10);
  pub_active_vols_ = create_publisher<std_msgs::msg::Float64MultiArray>("controller/active_volumes_ml", 1);
  pub_kpa_all_   = create_publisher<std_msgs::msg::Float64MultiArray>("controller/sensors_kpa", 10);
  pub_pos_dbg_   = create_publisher<std_msgs::msg::Float64MultiArray>("controller/position_dbg", 10);

  sub_volumes_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "actuator/volumes_ml", 10, std::bind(&Controller::on_volume, this, _1));

  sub_analog_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "board/analog", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(sensors_mtx_);
          // 액추에이터가 안 붙어 있으면 엔코더도 안 붙어 있다. 그때 board/analog 는
          // 전압이 안 잡히는 채널에 대해 0 raw 를 주는데, 그 값을 반전증폭 역산에
          // 넣으면 (4125−0)/0.825 = 5000 mV → offset·gain 에 따라 **엉뚱한 큰 각도**가
          // 나온다. 그 값이 속도 추정과 디버그 토픽으로 흘러 들어가 오해를 만든다.
          // 연결 전에는 0° 로 고정한다 (encoder_zero_when_disconnected 로 끌 수 있다).
          const bool zero = encoder_zero_when_disconnected_ && !actuator_connected_;
          const size_t n = std::min(msg->data.size(), encoder_angles_.size());
          for (size_t i = 0; i < n; ++i)
              encoder_angles_[i] = zero ? 0.0 : msg->data[i];
      });

  size_t nth = std::max<size_t>(2, std::min<size_t>(
      (size_t)num_total_channels_,
      std::thread::hardware_concurrency()));
  std::vector<int> pins; if (enable_thread_pinning_) for (auto v: cpu_pins_param_) pins.push_back((int)v);
  pool_threads_ = nth;
  pool_ = std::make_unique<ThreadPool>(nth, pins);

  build_mpcs();

  if (log_channel_id_ >= 0 && log_channel_id_ < num_total_channels_) {
    log_file_.open("mpc_log.csv", std::ios::out | std::ios::trunc);
    if (log_file_.is_open()) {
      RCLCPP_INFO(get_logger(), "Logging data for MPC channel %d to mpc_log.csv", log_channel_id_);
      log_file_ << "tick,reference_kpa,sensed_kpa\n";
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to open log file mpc_log.csv");
      log_channel_id_ = -1; 
    }
  }

  RCLCPP_INFO(get_logger(),
  "RefTcp: enable=%d host=%s port=%d expect_n=%d pressure_scale=%.6f",
  (int)ref_client_cfg_.enable, ref_client_cfg_.host.c_str(),
  ref_client_cfg_.port, ref_client_cfg_.expect_n, ref_client_cfg_.pressure_scale);

  // ──────────────────────────────────────────
  // 제어 모드 (0: 압력, 1: 위치)
  // ──────────────────────────────────────────
  control_mode_ = get_param_or<int>(this, "control_mode", 0);
  RCLCPP_INFO(get_logger(), "Control mode: %d (%s)",
              control_mode_, control_mode_ == 1 ? "POSITION" : "PRESSURE");

  // ──────────────────────────────────────────
  // 위치 제어기 파라미터 로드 (축마다 PositionController.axis<i>.* , 크기 = num_actuators_)
  // ──────────────────────────────────────────
  pos_ctrl_cfg_.assign(num_actuators_, PositionCtrlConfig{});
  pos_ctrl_state_.assign(num_actuators_, PositionCtrlState{});
  target_angle_deg_.assign(num_actuators_, 0.0);
  target_angle_slewed_.assign(num_actuators_, 0.0);
  target_slew_dps_ = get_param_or<double>(this, "PositionController.target_slew_deg_per_s", 0.0);
  target_follow_band_deg_ = get_param_or<double>(this,
      "PositionController.target_follow_band_deg", 5.0);
  kd_vel_ff_ = get_param_or<double>(this,
      "PositionController.kd_vel_ff", 1.0);
  target_slew_rate_.assign(std::max(1, num_actuators_), 0.0);
  band_sat_ticks_.assign(std::max(1, num_actuators_), 0);

  for (int a = 0; a < num_actuators_; ++a) {
    const std::string prefix = "PositionController.axis" + std::to_string(a) + ".";
    auto& c = pos_ctrl_cfg_[(size_t)a];

    // 공용 (mode 1 / 2)
    c.actuator_idx      = get_param_or<int>   (this, prefix + "actuator_idx",      a);
    c.pos_gid           = get_param_or<int>   (this, prefix + "pos_gid",           a);
    c.neg_gid           = get_param_or<int>   (this, prefix + "neg_gid",           num_positive_channels_ + a);
    c.mass_kg           = get_param_or<double>(this, prefix + "mass_kg",           1.0);
    c.link_length_m     = get_param_or<double>(this, prefix + "link_length_m",     0.2);
    c.p_pos_max_kpa     = get_param_or<double>(this, prefix + "p_pos_max_kpa",     165.0);
    c.p_neg_min_kpa     = get_param_or<double>(this, prefix + "p_neg_min_kpa",     70.0);
    c.vel_filter_alpha  = get_param_or<double>(this, prefix + "vel_filter_alpha",  0.05);
    c.default_angle_deg = get_param_or<double>(this, prefix + "default_angle_deg", 0.0);

    // mode 1 전용 (control_mode 2 에서는 읽히지 않는다)
    auto& h = c.m1;
    h.kp                 = get_param_or<double>(this, prefix + "mode1.kp",                 3.0);
    h.ki                 = get_param_or<double>(this, prefix + "mode1.ki",                 0.05);
    h.kd                 = get_param_or<double>(this, prefix + "mode1.kd",                 0.02);
    h.integral_limit_kpa = get_param_or<double>(this, prefix + "mode1.integral_limit_kpa", 20.0);
    h.kff_gravity        = get_param_or<double>(this, prefix + "mode1.kff_gravity",         10.0);
    h.friction_kpa       = get_param_or<double>(this, prefix + "mode1.friction_kpa",        2.0);
    h.p_bias_pos_kpa     = get_param_or<double>(this, prefix + "mode1.p_bias_pos_kpa",     120.0);
    h.p_bias_neg_kpa     = get_param_or<double>(this, prefix + "mode1.p_bias_neg_kpa",      90.0);
    h.neg_coupling       = get_param_or<double>(this, prefix + "mode1.neg_coupling",         0.5);
    h.p_pos_min_kpa      = get_param_or<double>(this, prefix + "mode1.p_pos_min_kpa",      101.325);
    h.p_neg_max_kpa      = get_param_or<double>(this, prefix + "mode1.p_neg_max_kpa",      101.325);
    h.ref_slew_kpa_per_s = get_param_or<double>(this, prefix + "mode1.ref_slew_kpa_per_s",   3.0);

    target_angle_deg_[(size_t)a] = c.default_angle_deg;

    RCLCPP_INFO(get_logger(),
      "[PosCtrl axis%d] 정격 P+≤%.1f / P-≥%.1f kPa | m=%.1fkg L=%.3fm | gid: pos=%d neg=%d enc=%d",
      a, c.p_pos_max_kpa, c.p_neg_min_kpa, c.mass_kg, c.link_length_m,
      c.pos_gid, c.neg_gid, c.actuator_idx);
    if (control_mode_ == 1)
      RCLCPP_INFO(get_logger(),
        "[PosCtrl axis%d] mode1 PID: kp=%.2f ki=%.3f kd=%.3f | kff=%.1f | bias P+=%.1f P-=%.1f",
        a, c.m1.kp, c.m1.ki, c.m1.kd, c.m1.kff_gravity,
        c.m1.p_bias_pos_kpa, c.m1.p_bias_neg_kpa);
  }

  // ──────────────────────────────────────────
  // 액추에이터 기하 (부피식 + 토크 환산의 단일 출처)
  // ──────────────────────────────────────────
  {
    const double dia_mm = get_param_or<double>(this, "Geometry.piston_dia_mm", 50.0);
    piston_area_mm2_    = M_PI * dia_mm * dia_mm / 4.0;
    reel_radius_mm_     = get_param_or<double>(this, "Geometry.reel_radius_mm",   25.0);
    vol_offset_pos_mm_  = get_param_or<double>(this, "Geometry.vol_offset_pos_mm", 40.0);
    vol_offset_neg_mm_  = get_param_or<double>(this, "Geometry.vol_offset_neg_mm", 90.0);
    RCLCPP_INFO(get_logger(),
      "Geometry: piston Ø%.1f mm (A=%.1f mm²), reel %.1f mm, offsets %.0f/%.0f mm",
      dia_mm, piston_area_mm2_, reel_radius_mm_, vol_offset_pos_mm_, vol_offset_neg_mm_);
  }

  // ──────────────────────────────────────────
  // 최적화 기반 압력 레퍼런스 생성기 (control_mode 2)
  // ──────────────────────────────────────────
  gen_period_ms_    = get_param_or<int>(this,  "PressureRefGen.period_ms", 20);
  gen_ref_slew_kpa_s_ = get_param_or<double>(this, "PressureRefGen.ref_slew_kpa_per_s", 150.0);
  gen_use_ej_meas_  = get_param_or<bool>(this, "PressureRefGen.use_ejector_measurement", true);
  gen_pos_ref_kpa_.assign(num_actuators_, sensor_.kpa_atm());
  gen_neg_ref_kpa_.assign(num_actuators_, sensor_.kpa_atm());
  gen_starve_pos_.assign(num_actuators_, 0.0);
  gen_starve_neg_.assign(num_actuators_, 0.0);

  tau_pid_.assign(num_actuators_, TorquePid{});
  tau_integ_.assign(num_actuators_, 0.0);
  for (int a = 0; a < num_actuators_; ++a) {
    const std::string pre = "TorquePID.axis" + std::to_string(a) + ".";
    auto& tp = tau_pid_[(size_t)a];
    tp.kp             = get_param_or<double>(this, pre + "kp",             0.0786);
    tp.ki             = get_param_or<double>(this, pre + "ki",             0.0295);
    tp.kd             = get_param_or<double>(this, pre + "kd",             0.0049);
    tp.integ_limit_nm = get_param_or<double>(this, pre + "integ_limit_nm", 2.0);
    tp.friction_nm    = get_param_or<double>(this, pre + "friction_nm",    0.30);
    tp.tau_ff_gain    = get_param_or<double>(this, pre + "tau_ff_gain",    1.0);
    tp.friction_band_deg = get_param_or<double>(this, pre + "friction_band_deg", 1.0);
  }

  {
    PressureRefGen::Params gp;
    gp.N  = num_actuators_;
    gp.dt = std::max(1e-3, gen_period_ms_ / 1000.0);
    gp.smooth_anchor_ref = get_param_or<bool>(this, "PressureRefGen.smooth_anchor_ref", true);
    const double A_m2 = piston_area_mm2_ * 1e-6;
    gp.Apos.assign(num_actuators_, A_m2);
    gp.Aneg.assign(num_actuators_, A_m2);

    // 채널 정격은 위치 제어기의 보수적 한계를 게이지 Pa 로 변환해 그대로 쓴다
    const double atm = sensor_.kpa_atm();
    const double pos_max_abs = pos_ctrl_cfg_.empty() ? 185.0   : pos_ctrl_cfg_[0].p_pos_max_kpa;
    const double neg_min_abs = pos_ctrl_cfg_.empty() ?  27.0   : pos_ctrl_cfg_[0].p_neg_min_kpa;
    gp.Pch_pos_max   = (pos_max_abs - atm) * 1000.0;
    gp.Pch_neg_min   = (neg_min_abs - atm) * 1000.0;
    // 레일 음압 셋포인트의 최대 깊이. 기본값은 채널 정격과 같지만 **별도 파라미터**다 —
    // 챔버가 필요한 깊이보다 레일을 더 깊게 요구하면 펌프 하나로 리저버+6챔버를 그
    // 깊이까지 뽑는 데 시간만 더 걸린다 (6축 정착시간의 지배 요인).
    gp.Pneg_cap_deep = get_param_or<double>(this, "PressureRefGen.rail.neg_sp_deep_kpa",
                                            neg_min_abs - atm) * 1000.0;

    gp.n_ch     = get_param_or<double>(this, "PressureRefGen.n_chamber", 1.4);
    gp.n_rail   = get_param_or<double>(this, "PressureRefGen.n_rail",    1.0);
    gp.Hpreview = get_param_or<int>   (this, "PressureRefGen.preview_steps", 1);
    gp.Pneg_shallow = get_param_or<double>(this, "PressureRefGen.rail.neg_shallow_kpa", -30.0) * 1000.0;
    gp.Ppos_sp_min  = get_param_or<double>(this, "PressureRefGen.rail.pos_sp_min_kpa",   30.0) * 1000.0;
    gp.Ppos_sp_max  = get_param_or<double>(this, "PressureRefGen.rail.pos_sp_max_kpa",  400.0) * 1000.0;
    gp.Fmax_ref     = get_param_or<double>(this, "PressureRefGen.rail.demand_ref_N",    150.0);
    gp.rail_pos_headroom = get_param_or<double>(this, "PressureRefGen.rail.pos_headroom_kpa", 60.0) * 1000.0;
    gp.rail_sp_decay_tau  = get_param_or<double>(this, "PressureRefGen.rail.sp_decay_tau_s", 2.0);
    gp.chamber_neg_headroom = get_param_or<double>(this, "PressureRefGen.rail.chamber_neg_headroom_kpa", 15.0) * 1000.0;
    gp.chamber_pos_headroom = get_param_or<double>(this, "PressureRefGen.rail.chamber_pos_headroom_kpa", 15.0) * 1000.0;
    gp.supply_filter_tau_s  = get_param_or<double>(this, "PressureRefGen.rail.supply_filter_tau_s", 0.5);
    gp.P_tank_stop  = get_param_or<double>(this, "PressureRefGen.tank_stop_kpa",        450.0) * 1000.0;

    gp.wtrack   = get_param_or<double>(this, "PressureRefGen.weights.track",  100.0);
    gp.w_flow   = get_param_or<double>(this, "PressureRefGen.weights.flow",     0.3);
    gp.w_smooth = get_param_or<double>(this, "PressureRefGen.weights.smooth",   0.5);
    gp.w_tank   = get_param_or<double>(this, "PressureRefGen.weights.tank",    15.0);
    gp.w_eject  = get_param_or<double>(this, "PressureRefGen.weights.eject",   25.0);
    gp.max_iter = get_param_or<int>   (this, "PressureRefGen.sqp_max_iter",     12);

    gp.Cd = get_param_or<double>(this, "PressureRefGen.Cd", 0.8);
    gp.valve_open_eta = get_param_or<double>(this, "PressureRefGen.valve_open_eta", 1.0);

    // 펌프 기하 — 키 이름을 시뮬(Virtual.pump.*)과 같게 맞춰 pump_params.yaml 하나로
    // 두 소비자를 동시에 갱신할 수 있게 한다. 이 블록이 없으면 생성기는 PistonPump.hpp
    // 하드코딩(예전 펌프)을 쓰고, 시뮬만 yaml 을 따라 **아무 경고 없이 어긋난다**.
    gp.pump.delta  = get_param_or<double>(this, "PressureRefGen.pump.delta_m",         gp.pump.delta);
    gp.pump.r      = get_param_or<double>(this, "PressureRefGen.pump.crank_m",         gp.pump.r);
    gp.pump.l      = get_param_or<double>(this, "PressureRefGen.pump.rod_m",           gp.pump.l);
    gp.pump.Spis   = get_param_or<double>(this, "PressureRefGen.pump.piston_area_m2",  gp.pump.Spis);
    gp.pump.Cb_out = get_param_or<double>(this, "PressureRefGen.pump.cb_out_m2",       gp.pump.Cb_out);
    gp.pump.Cb_in  = get_param_or<double>(this, "PressureRefGen.pump.cb_in_m2",        gp.pump.Cb_in);
    gp.pump.Npis   = get_param_or<int>   (this, "PressureRefGen.pump.n_piston",        gp.pump.Npis);
    gp.pump.omega  = get_param_or<double>(this, "PressureRefGen.pump.rpm",
                                          gp.pump.omega * 60.0 / (2.0 * M_PI)) * 2.0 * M_PI / 60.0;
    gp.pump_grid_n = get_param_or<int>(this, "PressureRefGen.pump_grid_n", gp.pump_grid_n);
    gp.set_orifices(
      get_param_or<double>(this, "PressureRefGen.orifice_mm.fill",   2.3),
      get_param_or<double>(this, "PressureRefGen.orifice_mm.vent",   4.0),
      get_param_or<double>(this, "PressureRefGen.orifice_mm.boost",  1.6),
      get_param_or<double>(this, "PressureRefGen.orifice_mm.suck",   4.0),
      get_param_or<double>(this, "PressureRefGen.orifice_mm.admit",  4.0),
      get_param_or<double>(this, "PressureRefGen.orifice_mm.eject",  4.0));

    refgen_ = std::make_unique<PressureRefGen>(gp);
    // 중앙집중 MPPI 는 이 생성기의 펌프 능력 테이블을 재사용하므로 **여기 뒤**에서
    // 만들어야 한다. build_mpcs() 직후에 만들었더니 refgen_ 이 아직 null 이어서
    // 펌프 유량이 0 이 됐고, 모델이 "양압 레일은 절대 회복 못 한다" 고 믿어
    // 레일 예측오차가 24.6 kPa 까지 벌어졌다.
    build_system_mppi();
    RCLCPP_INFO(get_logger(),
      "PressureRefGen: N=%d, dt=%.0f ms, 정격 P⁺≤%.1f kPa / P⁻≥%.1f kPa (gauge), "
      "Cd=%.2f eta=%.2f, F_max=%.1f N → τ_max=%.2f N·m",
      gp.N, gp.dt * 1e3, gp.Pch_pos_max / 1e3, gp.Pch_neg_min / 1e3,
      gp.Cd, gp.valve_open_eta,
      gp.Pch_pos_max * A_m2 + std::abs(gp.Pch_neg_min) * A_m2,
      (gp.Pch_pos_max + std::abs(gp.Pch_neg_min)) * A_m2 * reel_radius_mm_ * 1e-3);

    RCLCPP_INFO(get_logger(),
      "펌프 기하: delta=%.4f m r=%.4f l=%.4f Spis=%.4e Cb_out=%.3e Cb_in=%.3e "
      "rpm=%.0f Npis=%d  (소기량 %.2f mL, 사구간 %.3f mL, 압축비 %.1f)",
      gp.pump.delta, gp.pump.r, gp.pump.l, gp.pump.Spis, gp.pump.Cb_out, gp.pump.Cb_in,
      gp.pump.omega * 60.0 / (2.0 * M_PI), gp.pump.Npis,
      gp.pump.Spis * 2.0 * gp.pump.r * 1e6,
      gp.pump.Spis * (gp.pump.delta - 2.0 * gp.pump.r) * 1e6,
      (gp.pump.delta - 2.0 * gp.pump.r) > 1e-9
        ? gp.pump.delta / (gp.pump.delta - 2.0 * gp.pump.r) : -1.0);
    RCLCPP_INFO(get_logger(), "펌프 능력 테이블 계산 중...");
    const auto t0 = std::chrono::steady_clock::now();
    refgen_->build_pump_table();

    // 실측 능력경계(pump_fit_solve.py Phase F) — 기하 피팅보다 우선한다. 기하는
    // 5-파라미터 슬라이더-크랭크라 소기량×Cb_in 축퇴가 남아 데드헤드 외삽 오차가
    // 크다(자기검증 ~15%); 측정 구간 안은 직접 측정으로 덮어쓰고 밖은 기하 외삽을
    // 그대로 둔다. pump_frontier_measured 가 없으면(빈 벡터) 기하 테이블 그대로.
    {
      const auto pneg_kpa = get_param_or<std::vector<double>>(this,
          "PressureRefGen.pump_frontier_measured.pneg_kpa_gauge", {});
      const auto ppos_kpa = get_param_or<std::vector<double>>(this,
          "PressureRefGen.pump_frontier_measured.ppos_max_kpa_gauge", {});
      if (!pneg_kpa.empty() && pneg_kpa.size() == ppos_kpa.size()) {
        std::vector<double> pneg_pa(pneg_kpa.size()), ppos_pa(ppos_kpa.size());
        for (size_t i = 0; i < pneg_kpa.size(); ++i) {
          pneg_pa[i] = pneg_kpa[i] * 1000.0;
          ppos_pa[i] = ppos_kpa[i] * 1000.0;
        }
        refgen_->apply_measured_frontier(pneg_pa, ppos_pa);
        RCLCPP_INFO(get_logger(), "PressureRefGen: 실측 능력경계 %zu 점으로 cap_ppos 덮어씀 (Phase F)",
                    pneg_kpa.size());
      }
    }

    RCLCPP_INFO(get_logger(), "펌프 테이블 완료 (%.2f s). 능력경계: 음압 %.1f kPa → 양압 %.1f kPa",
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(),
      gp.Pneg_cap_deep / 1e3, refgen_->cap_ppos(gp.Pneg_cap_deep) / 1e3);
  }

  pub_refgen_dbg_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "controller/pressure_ref_dbg", 10);

  ref_server_cfg_.enable  = get_param_or<bool>(this, "RefTcpServer.enable",  false);
  ref_server_cfg_.port    = get_param_or<int> (this, "RefTcpServer.port",    2293);
  ref_server_cfg_.pos_gid = get_param_or<int> (this, "RefTcpServer.pos_gid", 0);
  ref_server_cfg_.neg_gid = get_param_or<int> (this, "RefTcpServer.neg_gid", num_positive_channels_);

  if (ref_server_cfg_.enable) {
    if (control_mode_ == 1 || control_mode_ == 2) {
      // 위치 제어 모드: TCP가 축 개수(num_actuators_)만큼의 angle_ref_deg 를 수신
      ref_server_cfg_.num_values = num_actuators_;
      ref_server_ = std::make_unique<RefTcpServer>(
        ref_server_cfg_,
        [this](const std::vector<double>& angles) {
          {
            std::lock_guard<std::mutex> lk(mpc_ref_mtx_);
            for (size_t i = 0; i < angles.size() && i < target_angle_deg_.size(); ++i)
              target_angle_deg_[i] = angles[i];
            pos_tcp_received_ = true;
          }
          std::string s;
          for (double a : angles) s += (s.empty() ? "" : ", ") + std::to_string(a);
          RCLCPP_INFO(rclcpp::get_logger("RefTcpServer"), "[PosCtrl] angle_ref = [%s] deg", s.c_str());
        }
      );
      RCLCPP_INFO(get_logger(),
        "RefTcpServer [POSITION mode]: port %d — expects [%d doubles: angle_ref_deg per axis]",
        ref_server_cfg_.port, num_actuators_);
    } else {
      // 압력 제어 모드: TCP가 [pos_kpa, neg_kpa] 수신 (기존 동작)
      ref_server_cfg_.num_values = 2;
      ref_server_ = std::make_unique<RefTcpServer>(
        ref_server_cfg_,
        [this](const std::vector<double>& v) {
          if (v.size() < 2) return;
          std::lock_guard<std::mutex> lk(mpc_ref_mtx_);
          const int pg = ref_server_cfg_.pos_gid;
          const int ng = ref_server_cfg_.neg_gid;
          if (pg >= 0 && pg < (int)mpc_ref_kpa_.size()) mpc_ref_kpa_[pg] = v[0];
          if (ng >= 0 && ng < (int)mpc_ref_kpa_.size()) mpc_ref_kpa_[ng] = v[1];
        }
      );
      RCLCPP_INFO(get_logger(),
        "RefTcpServer [PRESSURE mode]: port %d (pos_gid=%d, neg_gid=%d) — expects [double pos_kpa, double neg_kpa]",
        ref_server_cfg_.port, ref_server_cfg_.pos_gid, ref_server_cfg_.neg_gid);
    }
  }

  filt_state_.assign(NUM_CAN_BOARDS, 101.325);

  zero_calib_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/zero_calibration",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr req,
           std_srvs::srv::Trigger::Response::SharedPtr res) {
      on_zero_calibration(req, res);
    });

  pressure_safety_limit_kpa_      = get_param_or<double>(this, "pressure_safety_limit_kpa",           170.0);
  pressure_safety_hysteresis_kpa_ = get_param_or<double>(this, "pressure_safety_hysteresis_kpa",       10.0);
  RCLCPP_INFO(get_logger(), "Over-pressure safety: limit=%.1f kPa, hysteresis=%.1f kPa (release at %.1f kPa)",
    pressure_safety_limit_kpa_, pressure_safety_hysteresis_kpa_,
    pressure_safety_limit_kpa_ - pressure_safety_hysteresis_kpa_);

  start_time_ = std::chrono::steady_clock::now();
  elapsed_time_sec_ = 0.0;

  RCLCPP_INFO(this->get_logger(), "Controller node initialization complete.");
}

namespace {
// 파라미터 이름 → ControlAug 필드. 선언과 콜백이 **같은 목록**을 쓰게 해서
// 새 항목을 넣을 때 한 곳만 고치면 되게 한다.
struct AugBind { const char* name; bool is_bool; };
}  // namespace

void Controller::declare_aug_params() {
  auto& a = aug_;
  // 기본값은 전부 off / 현재 동작 유지. 아무것도 켜지 않으면 기존과 동일하다.
  a.adapt_gain   = get_param_or<bool>  (this, "aug.adapt_gain",   false);
  a.adapt_rate   = (float)get_param_or<double>(this, "aug.adapt_rate",   0.20);
  a.gain_min     = (float)get_param_or<double>(this, "aug.gain_min",     0.25);
  a.gain_max     = (float)get_param_or<double>(this, "aug.gain_max",     4.00);
  a.adapt_min_flow_lpm = (float)get_param_or<double>(this, "aug.adapt_min_flow_lpm", 0.10);
  a.adapt_window = get_param_or<int>(this, "aug.adapt_window", 100);

  a.offset_free  = get_param_or<bool>  (this, "aug.offset_free",  false);
  a.dist_rate    = (float)get_param_or<double>(this, "aug.dist_rate",    0.5);
  a.dist_band_kpa= (float)get_param_or<double>(this, "aug.dist_band_kpa",  5.0);
  a.dist_limit_kpa    = (float)get_param_or<double>(this, "aug.dist_limit_kpa",    30.0);
  a.dist_deadband_kpa = (float)get_param_or<double>(this, "aug.dist_deadband_kpa",  0.3);

  a.auto_tune    = get_param_or<bool>  (this, "aug.auto_tune",    false);
  a.tune_rate    = (float)get_param_or<double>(this, "aug.tune_rate",    0.02);
  a.tau_min      = (float)get_param_or<double>(this, "aug.tau_min",      0.06);
  a.tau_max      = (float)get_param_or<double>(this, "aug.tau_max",      0.40);
  a.osc_hi       = (float)get_param_or<double>(this, "aug.osc_hi",       0.30);
  a.err_slow_kpa = (float)get_param_or<double>(this, "aug.err_slow_kpa", 3.0);
  a.tune_window  = get_param_or<int>   (this, "aug.tune_window",  250);

  // **재시작 없이** 바꿀 수 있게 한다. 하나씩 켜 가며 효과를 분리해 보는 것이 요점이다.
  aug_cb_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter>& ps) {
      rcl_interfaces::msg::SetParametersResult r; r.successful = true;
      std::lock_guard<std::mutex> lk(aug_mtx_);
      for (const auto& q : ps) {
        const std::string& n = q.get_name();
        if (n.rfind("aug.", 0) != 0) continue;
        try {
          if      (n == "aug.adapt_gain")   aug_.adapt_gain  = q.as_bool();
          else if (n == "aug.offset_free")  aug_.offset_free = q.as_bool();
          else if (n == "aug.auto_tune")    aug_.auto_tune   = q.as_bool();
          else if (n == "aug.adapt_rate")   aug_.adapt_rate  = (float)q.as_double();
          else if (n == "aug.gain_min")     aug_.gain_min    = (float)q.as_double();
          else if (n == "aug.gain_max")     aug_.gain_max    = (float)q.as_double();
          else if (n == "aug.adapt_min_flow_lpm") aug_.adapt_min_flow_lpm = (float)q.as_double();
          else if (n == "aug.adapt_window") aug_.adapt_window = (int)q.as_int();
          else if (n == "aug.dist_rate")    aug_.dist_rate   = (float)q.as_double();
          else if (n == "aug.dist_band_kpa") aug_.dist_band_kpa = (float)q.as_double();
          else if (n == "aug.dist_limit_kpa")    aug_.dist_limit_kpa    = (float)q.as_double();
          else if (n == "aug.dist_deadband_kpa") aug_.dist_deadband_kpa = (float)q.as_double();
          else if (n == "aug.tune_rate")    aug_.tune_rate   = (float)q.as_double();
          else if (n == "aug.tau_min")      aug_.tau_min     = (float)q.as_double();
          else if (n == "aug.tau_max")      aug_.tau_max     = (float)q.as_double();
          else if (n == "aug.osc_hi")       aug_.osc_hi      = (float)q.as_double();
          else if (n == "aug.err_slow_kpa") aug_.err_slow_kpa= (float)q.as_double();
          else if (n == "aug.tune_window")  aug_.tune_window = (int)q.as_int();
        } catch (const std::exception& e) {
          r.successful = false; r.reason = e.what();
        }
      }
      RCLCPP_INFO(get_logger(), "보강 갱신: 이득적응=%s 오프셋프리=%s 자동튜닝=%s",
                  aug_.adapt_gain ? "on" : "off", aug_.offset_free ? "on" : "off",
                  aug_.auto_tune ? "on" : "off");
      return r;
    });

  RCLCPP_INFO(get_logger(),
    "제어 보강 (ros2 param set 으로 실행 중 변경 가능): "
    "aug.adapt_gain=%s aug.offset_free=%s aug.auto_tune=%s",
    a.adapt_gain ? "on" : "off", a.offset_free ? "on" : "off", a.auto_tune ? "on" : "off");
}

void Controller::push_aug_to_mpcs() {
  ControlAug snap;
  { std::lock_guard<std::mutex> lk(aug_mtx_); snap = aug_; }
  for (auto& m : mpcs_) m->cfg_mutable().aug = snap;
}

void Controller::on_zero_calibration(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  sensor_zero_sum_.fill(0.0);
  sensor_zero_cnt_.fill(0);
  sensor_zero_tick_ = 0;
  sensor_zeroed_    = false;
  RCLCPP_INFO(get_logger(), "Sensor zero-calibration re-triggered (current values → offset).");
  res->success = true;
  res->message = "Zero-calibration started. Offsets will update in ~0.5 sec.";
}

Controller::~Controller()
{
  if (log_file_.is_open()) {
    log_file_.close();
    RCLCPP_INFO(get_logger(), "Log file mpc_log.csv closed.");
  }
}

void Controller::build_mpcs() {
  // 활성 채널은 **축별 gid 설정**에서 온다. 예전에는 0..num_actuators-1 로
  // 하드코딩돼 있어서, PositionController.axisN.pos_gid 를 바꿔도 그 채널의
  // MPC 가 만들어지지 않아 무시됐다.
  //
  // 이 덕분에 축 하나로 임의의 물리 채널을 돌릴 수 있다 (채널별 부피·밸브를
  // 하나씩 재려면 필수다 — 여럿을 같이 돌리면 레일을 나눠 쓰느라 차압이 흔들려
  // 측정이 흩어진다).
  //   예: ch2 만 → num_actuators=1, axis0.pos_gid=2, axis0.neg_gid=8,
  //               axis0.actuator_idx=2   (control.launch.py 의 axis:=2 가 이걸 한다)
  //
  // 주의: pos_ctrl_cfg_ 는 이 함수보다 **뒤에** 로드되므로 파라미터를 직접 읽는다.
  active_channels_.clear();
  for (int i = 0; i < num_actuators_; ++i) {
    const std::string pfx = "PositionController.axis" + std::to_string(i) + ".";
    const int pg = get_param_or<int>(this, pfx + "pos_gid", i);
    const int ng = get_param_or<int>(this, pfx + "neg_gid", num_positive_channels_ + i);
    if (pg >= 0 && pg < num_total_channels_) active_channels_.insert(pg);
    else RCLCPP_ERROR(get_logger(), "axis%d.pos_gid=%d 가 범위 밖이다 (0~%d)",
                      i, pg, num_total_channels_ - 1);
    if (ng >= 0 && ng < num_total_channels_) active_channels_.insert(ng);
    else RCLCPP_ERROR(get_logger(), "axis%d.neg_gid=%d 가 범위 밖이다 (0~%d)",
                      i, ng, num_total_channels_ - 1);
  }
  {
    std::string s;
    for (int gid : active_channels_) { char b[16]; snprintf(b, sizeof(b), " %d", gid); s += b; }
    RCLCPP_INFO(get_logger(), "[활성 채널 gid]%s  (보드 = gid + %d)",
                s.c_str(), channel_board_offset_);
  }
  const std::set<int>& active_channels = active_channels_;

  mpcs_.clear();
  mpcs_.reserve(active_channels.size());

  auto ml_to_m3 = [](double ml){ return ml * 1e-6; };


  for (int gid = 0; gid < num_total_channels_; ++gid) {
      if (active_channels.find(gid) == active_channels.end()) {
          continue;
      }

      AcadosMpc::Config cfg;
      cfg.can_board_id = gid + channel_board_offset_;   // e.g. gid 0 → board 4
      cfg.global_id    = gid;

      cfg.NP = mpc_.NP; cfg.n_x = mpc_.n_x; cfg.n_u = mpc_.n_u; cfg.Ts = (float)mpc_.Ts;
      cfg.Q_value = (float)mpc_.Q_value; cfg.R_value = (float)mpc_.R_value;
      cfg.A_lin = 1.0f;
      cfg.B_lin = {1.0f, 0.5f, -0.8f};

      // [수정됨] Config에서 설정한 num_positive_channels_ 변수 사용
      cfg.is_positive   = (gid < num_positive_channels_); 
      
      cfg.pos_ki_micro  = (float)channel_configs_[gid].pos_ki_micro;
      cfg.pos_ki_macro  = (float)channel_configs_[gid].pos_ki_macro;
      cfg.pos_ki_atm    = (float)channel_configs_[gid].pos_ki_atm;
      cfg.neg_ki_micro  = (float)channel_configs_[gid].neg_ki_micro;
      cfg.neg_ki_macro  = (float)channel_configs_[gid].neg_ki_macro;
      cfg.neg_ki_atm    = (float)channel_configs_[gid].neg_ki_atm;

      // 밸브별 13-parameter → cfg.pv[3]. 채널 공통 필드는 세 원소에 같은 값을 넣는다.
      for (int j = 0; j < 3; ++j) {
        const auto& t = channel_configs_[gid].v[(size_t)j];
        auto& d = cfg.pv[(size_t)j];
        d.I_MAX = (float)t.I_MAX;             d.A_max = (float)t.A_max;
        d.k_shape = (float)t.k_shape;         d.C_k = (float)t.C_k;
        d.C_p = (float)t.C_p;                 d.C_z = (float)t.C_z;
        d.A_bw = (float)t.A_bw;               d.beta_bw = (float)t.beta_bw;
        d.gamma_bw = (float)t.gamma_bw;       d.alpha_shape = (float)t.alpha_shape;
        d.wn_up = (float)t.wn_up;             d.zeta_up = (float)t.zeta_up;
        d.wn_down = (float)t.wn_down;         d.zeta_down = (float)t.zeta_down;
        d.is_positive     = (gid < num_positive_channels_);
        d.ejector_p_limit = (float)mpc_.ejector_p_limit;
        d.leakage_u       = (float)((gid < num_positive_channels_) ? mpc_.leakage_u_pos
                                                                  : mpc_.leakage_u_neg);
        d.crack_area_frac = (float)mpc_.valve_crack_area_frac;
        d.finalize();
      }
      // ── 밸브 제어권(authority) 검사 ─────────────────────────────────────
      // 정상폐쇄 밸브는 상류압이 올라갈수록 C_p·Pin 항 때문에 열리기 쉬워진다.
      // 그 항이 스프링 예압을 이겨 버리는 상류압을 넘으면 **u=0 에서도 원시 모델이
      // 열려 있다** — 즉 그 압력 위에서는 밸브를 닫을 수 없고 제어권이 없다.
      // (area_eff 가 받침을 빼 주므로 폭주하지는 않지만, 그 구간에서 13-parameter
      //  는 데이터 밖 외삽이라 예측이 맞지 않는다.)
      //
      // 실기 피팅(config/valve_params.yaml)이 정확히 이 상태다: C_p 가 탐색 상한
      // 2.0e-3 에 걸린 채 끝나 micro 의 한계가 135 kPa abs 로 나온다. mode 2 의
      // 레일 셋포인트는 최대 351 kPa abs 이므로 그 위에서 돌게 된다.
      // → 압력을 바꿔 가며 스텝을 주는 재피팅이 필요하다 (RUNBOOK 밸브 절).
      {
        const double rail_max_abs = 101.325
            + get_param_or<double>(this, "PressureRefGen.rail.pos_sp_max_kpa", 250.0);
        const double chamber_max_abs =
            get_param_or<double>(this, "pressure_safety_limit_kpa", 190.0);
        static const char* kRole[3] = {"micro", "macro", "atm"};
        for (int j = 0; j < 3; ++j) {
          const auto& d = cfg.pv[(size_t)j];
          if (d.C_p <= 1e-12f) continue;                    // 압력 의존 없음 = 항상 닫힌다
          // 이 밸브가 실제로 볼 수 있는 최대 상류압
          // 이 밸브가 실제로 겪는 최대 상류압:
          //   양압 micro ← 레일 (컨트롤러가 직접 셋포인트를 정한다)
          //   그 외      ← 챔버 (과압 세이프티 한계가 상한이다)
          // 양압 macro 의 상류(탱크)는 컨트롤러 설정에 없어 챔버 기준으로만 본다.
          const double p_seen = (j == mppi::V_MICRO && gid < num_positive_channels_)
                              ? rail_max_abs : chamber_max_abs;

          // 예전에는 F_open(A_eff 가 float 0 으로 언더플로하는 점) 으로 한계압을
          // 구했다. 그 기준은 alpha_shape 가 클 때만 뜻이 있다 — alpha=1 이면
          // 시그모이드가 0 에 닿지 않아 F_open 이 무한히 멀어지고 모든 밸브가
          // 무조건 걸린다 (실제로 12채널 × 3밸브 전부 ERROR 를 뱉었다).
          //
          // 물어야 할 것은 "언제 언더플로하나" 가 아니라 "이 밸브가 겪는 최대
          // 상류압에서 u=0 받침이 A_max 대비 얼마나 큰가" 다. 받침이 무시할
          // 수준이면 (area_eff 가 빼 주므로) 아무 문제가 없고, 유의미하게 크면
          // 그 구간은 데이터 밖 외삽이라 예측을 믿을 수 없다.
          const double F0 = (double)d.C_p * p_seen - (double)d.C_k;
          const double x  = (double)d.k_shape * F0;
          const double lg = (x > 0.0) ? std::log1p(std::exp(-x)) : (-x + std::log1p(std::exp(x)));
          const double frac = std::exp(-(double)d.alpha_shape * lg);   // = sigmoid(x)^alpha
          constexpr double kPedestalWarn = 0.01;            // A_max 의 1%
          if (frac > kPedestalWarn) {
            RCLCPP_ERROR(get_logger(),
              "[밸브 검증] ch%d.%s: 최대 상류압 %.0f kPa abs 에서 u=0 받침이 "
              "A_max 의 %.1f%% 다 (제어권 상실). C_p=%.3e 가 피팅 상한(2.0e-3)에 "
              "걸렸는지 확인하고 상류압을 바꿔 가며 재피팅할 것. 지금은 받침을 "
              "빼서 안전하게 돌지만 그 구간 예측은 외삽이다.",
              gid, kRole[j], p_seen, 100.0 * frac, (double)d.C_p);
          }
        }
      }

      // 하위 호환용 평면 필드 = micro 밸브
      const auto& m0 = channel_configs_[gid].v[0];
      cfg.I_MAX = (float)m0.I_MAX;           cfg.A_max = (float)m0.A_max;
      cfg.k_shape = (float)m0.k_shape;       cfg.C_k = (float)m0.C_k;
      cfg.C_p = (float)m0.C_p;               cfg.C_z = (float)m0.C_z;
      cfg.A_bw = (float)m0.A_bw;             cfg.beta_bw = (float)m0.beta_bw;
      cfg.gamma_bw = (float)m0.gamma_bw;     cfg.alpha_shape = (float)m0.alpha_shape;
      cfg.wn_up = (float)m0.wn_up;           cfg.zeta_up = (float)m0.zeta_up;
      cfg.wn_down = (float)m0.wn_down;       cfg.zeta_down = (float)m0.zeta_down;


      cfg.ref_value = 101.325f;
      cfg.du_min = -30.f; cfg.du_max = +30.f;
      cfg.u_abs_min = 0.f; cfg.u_abs_max = 100.f;

      cfg.volume_m3 = ml_to_m3(vol_ml_[gid]);

      cfg.ejector_k       = (float)mpc_.ejector_k;
      cfg.ejector_p_limit = (float)mpc_.ejector_p_limit;

      cfg.leakage_u_pos = (float)mpc_.leakage_u_pos;
      cfg.leakage_u_neg = (float)mpc_.leakage_u_neg;

      cfg.target_time_constant = (float)mpc_.target_tc;
      cfg.valve_crack_area_frac = (float)mpc_.valve_crack_area_frac;
      cfg.cmd_lpf_hz            = (float)mpc_.cmd_lpf_hz;
      cfg.ki_u_limit_pct        = (float)mpc_.ki_u_limit_pct;
      cfg.ki_flow               = (float)mpc_.ki_flow;
      cfg.q_trim_limit          = (float)mpc_.q_trim_limit;
      cfg.crack_floor_rate_kpas = (float)mpc_.crack_floor_rate_kpas;
      cfg.crack_floor_min_err_kpa = (float)mpc_.crack_floor_min_err_kpa;
      cfg.integ_hold_rate_kpas    = (float)mpc_.integ_hold_rate_kpas;
      cfg.integ_deadzone_boost    = (float)mpc_.integ_deadzone_boost;

      // mppi_system 에서는 채널 솔버를 만들지 않는다 — Δu 를 중앙집중이 낸다.
      // prepare/finish(피드포워드·적분·크래킹·상태추정)는 그대로 쓰인다.
      cfg.use_mppi             = (mpc_.solver == "mppi");
      cfg.mppi_samples         = mpc_.mppi_samples;
      cfg.mppi_lambda          = (float)mpc_.mppi_lambda;
      cfg.mppi_sigma_pct       = (float)mpc_.mppi_sigma_pct;
      cfg.mppi_sigma_explore_pct = (float)mpc_.mppi_sigma_explore_pct;
      cfg.mppi_explore_frac      = (float)mpc_.mppi_explore_frac;
      cfg.mppi_du_limit_pct      = (float)mpc_.mppi_du_limit_pct;
      cfg.mppi_ref_tau_s         = (float)mpc_.mppi_ref_tau_s;
      cfg.mppi_np                = mpc_.mppi_np;
      cfg.mppi_ts_s              = (float)mpc_.mppi_ts_s;
      cfg.mppi_noise_beta      = (float)mpc_.mppi_noise_beta;
      cfg.mppi_w_track         = (float)mpc_.mppi_w_track;
      cfg.mppi_w_effort        = (float)mpc_.mppi_w_effort;
      cfg.mppi_w_du            = (float)mpc_.mppi_w_du;
      cfg.mppi_track_scale_kpa = (float)mpc_.mppi_track_scale_kpa;
      cfg.mppi_terminal_mult   = (float)mpc_.mppi_terminal_mult;
      cfg.mppi_substeps        = mpc_.mppi_substeps;
      cfg.mppi_raw_state       = mpc_.mppi_raw_state;
      cfg.mppi_estimator       = mpc_.mppi_estimator;
      cfg.obs_gain             = (float)mpc_.obs_gain;
      cfg.obs_bridge_alpha     = (float)mpc_.obs_bridge_alpha;
      // 컨트롤러 LPF 계수는 이미 알고 있으므로 그대로 넘긴다 (복제 정확도).
      cfg.obs_ctrl_alpha       = (float)sensor_filter_alpha_;

      auto mpc_obj = std::make_unique<AcadosMpc>(cfg);
      // MPPI 를 써도 QP 솔버는 붙여 둔다 — solver 파라미터만 바꿔 같은 빌드로
      // A/B 비교할 수 있어야 하고, 붙어 있어도 호출되지 않으면 비용이 0 이다.
      int nv = cfg.n_u * cfg.NP; 
      int nc = 0; 
      auto qp_solver = std::make_shared<QP>(nv, nc);
      mpc_obj->set_qp_solver(qp_solver);

      mpcs_.emplace_back(std::move(mpc_obj));
  }

  RCLCPP_INFO(get_logger(), "Initialized %zu MPC controllers based on active_mpc_channels parameter.", mpcs_.size());
  declare_aug_params();
  {
    // 실기에서 **피팅 결과가 실제로 로드됐는지** 확인하는 유일한 수단이다.
    // valve_params.yaml 을 config/ 에 넣고 재빌드했는데 0/N 이 나오면 병합이 안 된 것이다.
    int n_pv = 0;
    for (int gid : active_channels_)
      if (gid >= 0 && gid < (int)channel_configs_.size()
          && channel_configs_[(size_t)gid].per_valve_loaded) ++n_pv;
    RCLCPP_INFO(get_logger(),
      "밸브별 13-parameter: %d/%zu 채널이 chN.{micro,atm,macro}.* 를 로드했다%s",
      n_pv, active_channels_.size(),
      n_pv == 0 ? " — 평면 chN.* 또는 기본값 사용 (피팅 전이면 정상)" : "");
    for (int gid : active_channels_) {
      if (gid < 0 || gid >= (int)channel_configs_.size()) continue;
      const double v = channel_configs_[(size_t)gid].chamber_volume_ml;
      if (v > 0.0)
        RCLCPP_INFO(get_logger(), "  ch%d 피팅 챔버 부피 %.2f mL", gid, v);
    }
  }
  if (mpc_.solver == "mppi") {
    RCLCPP_INFO(get_logger(),
      "MPC 솔버 = MPPI (선형화 없음): K=%d, NP=%d, Ts=%.1f ms (지평 %.0f ms), 서브스텝=%d, "
      "lambda=%.3f(비용 산포 비율), sigma=%.1f%%, beta=%.2f, "
      "w=(track %.3g, effort %.3g, du %.3g), 오차 기준 %.1f kPa, 말단 ×%.1f",
      mpc_.mppi_samples,
      (mpc_.mppi_np > 0 ? mpc_.mppi_np : mpc_.NP),
      (mpc_.mppi_ts_s > 0.0 ? mpc_.mppi_ts_s : mpc_.Ts) * 1000.0,
      (mpc_.mppi_np > 0 ? mpc_.mppi_np : mpc_.NP)
        * (mpc_.mppi_ts_s > 0.0 ? mpc_.mppi_ts_s : mpc_.Ts) * 1000.0,
      mpc_.mppi_substeps,
      mpc_.mppi_lambda, mpc_.mppi_sigma_pct, mpc_.mppi_noise_beta,
      (mpc_.mppi_w_track  >= 0.0 ? mpc_.mppi_w_track  : mpc_.Q_value),
      (mpc_.mppi_w_effort >= 0.0 ? mpc_.mppi_w_effort : mpc_.R_value),
      mpc_.mppi_w_du, mpc_.mppi_track_scale_kpa, mpc_.mppi_terminal_mult);
  } else {
    RCLCPP_INFO(get_logger(), "MPC 솔버 = QP (선형화 + 응축 qpOASES)");
  }

  zoh_.fill(0);
}


void Controller::on_sensor(const std_msgs::msg::UInt16MultiArray::SharedPtr m) {
  {
    std::lock_guard<std::mutex> lk(sensors_mtx_);
    const size_t n = std::min(m->data.size(), sensors_raw_.size());
    for (size_t i = 0; i < n; ++i) sensors_raw_[i] = m->data[i];
  }
  on_timer();
}

// 각도 목표를 슬루 제한으로 램프시킨다.
//
// TCP 로 들어온 목표를 계단으로 주면 레퍼런스 생성기가 즉시 큰 힘을 요구하고
// 액추에이터가 그만큼 세게 튄다. 목표를 램프시키면 압력 레퍼런스도 따라서
// 완만해진다 — 액추에이터를 보수적으로 움직일 때 여기부터 조인다.
// target_slew_deg_per_s <= 0 이면 계단 그대로다.
void Controller::slew_targets(double dt_sec) {
  // ── 기동 시: 슬루 상태를 **현재 각도에서 한 번만** 출발시킨다 ────────────
  //
  // 목표 자체는 처음부터 default_angle_deg (= 0) 다. 시작하면 모든 축이 0° 로
  // 내려온다. 다만 **출발점**은 지금 팔이 있는 각도여야 한다 — 슬루 상태를 0 에서
  // 시작하면 첫 틱에 angle_ref 가 측정각에서 0 으로 계단 점프한다.
  // 실기 20260829_165306: t=10.79 에 각도 19.9° 인데 목표가 0.45° 로 떨어졌다
  // (−19.4° 계단) — 액추에이터에 그대로 충격으로 간다.
  //
  // 한 번 씨앗을 심고 나면 평범하게 target_slew_deg_per_s 로 0 까지 램프한다.
  // (예전에는 TCP 명령이 올 때까지 계속 측정각에 붙여 뒀다. 그러면 목표가 팔을
  //  따라다니기만 하고 0 으로 안 갔다.)
  if (!slew_seeded_) {
    std::array<double, 9> ang;
    { std::lock_guard<std::mutex> lk(sensors_mtx_); ang = encoder_angles_; }
    // 엔코더가 아직 안 들어왔으면(전부 0) 다음 틱에 다시 시도한다 — 0 에서
    // 출발했다가 진짜 각도가 들어오는 순간 계단이 되는 것을 막는다.
    bool have = false;
    for (size_t i = 0; i < pos_ctrl_cfg_.size() && !have; ++i) {
      const int enc = std::clamp(pos_ctrl_cfg_[i].actuator_idx, 0, (int)ang.size() - 1);
      if (std::abs(ang[(size_t)enc]) > 1e-9) have = true;
    }
    if (target_angle_slewed_.size() != target_angle_deg_.size())
      target_angle_slewed_.assign(target_angle_deg_.size(), 0.0);
    for (size_t i = 0; i < target_angle_slewed_.size(); ++i) {
      const int enc = (i < pos_ctrl_cfg_.size())
          ? std::clamp(pos_ctrl_cfg_[i].actuator_idx, 0, (int)ang.size() - 1) : (int)i;
      target_angle_slewed_[i] = ang[(size_t)enc];
    }
    target_slew_rate_.assign(target_angle_slewed_.size(), 0.0);
    if (have || slew_seed_ticks_++ > 500) {   // 500 틱(=1 s @500 Hz) 지나면 0 으로 확정
      slew_seeded_ = true;
      std::string vs;
      for (size_t i = 0; i < target_angle_slewed_.size(); ++i) {
        char b[32]; snprintf(b, sizeof(b), " ax%zu=%.1f°", i, target_angle_slewed_[i]); vs += b;
      }
      RCLCPP_INFO(get_logger(),
        "기동 목표: 전 축 %.1f° 로 내려간다 (%.0f deg/s). 현재 각도에서 출발:%s",
        target_angle_deg_.empty() ? 0.0 : target_angle_deg_[0], target_slew_dps_, vs.c_str());
    }
    return;
  }

  if (target_slew_dps_ <= 0.0 || dt_sec <= 0.0) {
    target_angle_slewed_ = target_angle_deg_;
    target_slew_rate_.assign(target_angle_slewed_.size(), 0.0);   // 계단이면 속도 FF 없음
    return;
  }
  if (target_angle_slewed_.size() != target_angle_deg_.size())
    target_angle_slewed_ = target_angle_deg_;
  if (target_slew_rate_.size() != target_angle_slewed_.size())
    target_slew_rate_.assign(target_angle_slewed_.size(), 0.0);
  const std::vector<double> prev = target_angle_slewed_;
  const double step = target_slew_dps_ * dt_sec;
  for (size_t i = 0; i < target_angle_deg_.size(); ++i) {
    const double d = target_angle_deg_[i] - target_angle_slewed_[i];
    target_angle_slewed_[i] += std::clamp(d, -step, step);
  }

  // 목표 슬루 **속도** 를 남긴다. 제어기의 D 항이 이 값을 피드포워드로 쓴다 —
  // 그래야 D 가 "명령한 움직임" 이 아니라 "명령에서 벗어난 만큼" 만 억제한다.
  for (size_t i = 0; i < target_angle_slewed_.size(); ++i)
    target_slew_rate_[i] = (target_angle_slewed_[i] - prev[i]) / dt_sec;
}

// 토픽: actuator/volumes_ml  (Float64MultiArray, num_total_channels_ 개, 단위 mL)
// 활성 채널만 업데이트. 비활성 채널은 default_volume_ml_ 유지.
void Controller::on_volume(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (!actuator_connected_) return;
  const int n = std::min((int)msg->data.size(), num_total_channels_);
  for (int i = 0; i < n; ++i) {
    if (active_channels_.count(i) == 0) continue;
    if (msg->data[i] > 0.0) vol_ml_[i] = msg->data[i];
  }
}

// ============================================================================
// 중앙집중 MPPI — 12채널 + 라인 밸브 2개를 하나의 최적화로 푼다
//
// 채널별 독립 MPPI 는 공유 레일을 지평 동안 상수로 둔다. 실제로는 채널 1개 개방에
// ≈5 kPa, 양압 6채널 동시에 ≈34 kPa 강하한다 (추종 정규화 기준 10 kPa 대비 0.5~3.4σ).
// 게다가 "채널 수요 합 vs 레일 공급" 을 비교하는 코드가 어디에도 없었다.
// 전체를 한 모델에 넣으면 그 결합이 자동으로 들어오고 라인 PID 도 흡수된다.
//
// **기존 자산을 한 줄도 복제하지 않는다**: 채널별 피드포워드·적분항·크래킹 임계·
// macro 게이트는 AcadosMpc::prepare/finish 를 그대로 부른다. 이 함수는 그 사이에서
// Δu 만 전체 시스템 기준으로 다시 결정한다.
// ============================================================================
void Controller::build_system_mppi()
{
  if (mpc_.solver != "mppi_system") return;
  if (mpcs_.empty()) {
    RCLCPP_ERROR(get_logger(), "mppi_system: 활성 채널이 없다");
    return;
  }

  auto& sp = sys_params_;
  sp.n_ch  = num_total_channels_;
  sp.n_pos = num_positive_channels_;
  sp.P_atm = (float)sensor_.kpa_atm();

  // 채널별 13-parameter — AcadosMpc 가 이미 만들어 둔 것을 그대로 쓴다 (단일 출처).
  sp.ch.assign((size_t)sp.n_ch, mppi::ChannelPlant{});
  for (auto& m : mpcs_) {
    const int gid = m->cfg().global_id;
    if (gid >= 0 && gid < sp.n_ch) sp.ch[(size_t)gid] = m->plant_params();
  }
  // 비활성 채널은 밸브를 닫아 둔 것과 같으므로 방향만 맞춰 둔다.
  for (int g = 0; g < sp.n_ch; ++g)
    for (auto& t : sp.ch[(size_t)g]) t.is_positive = (g < sp.n_pos);

  // 라인 밸브 — 채널 0 micro 의 13-parameter 를 공용으로 쓴다. 실기에서는 라인 밸브를
  // 따로 피팅해야 한다 (RUNBOOK.md 는 채널 36개만 다룬다).
  sp.line = sp.ch.empty() ? mppi::PlantParams{} : sp.ch[0][mppi::V_MICRO];
  sp.line.leakage_u = 0.0f;
  sp.line.is_positive = true;      // 사용처에서 상·하류를 직접 지정하므로 무의미하다

  sp.V_pos_m3 = (float)(get_param_or<double>(this, "MPC_parameters.rail_volume_pos_ml", 500.0) * 1e-6);
  sp.V_neg_m3 = (float)(get_param_or<double>(this, "MPC_parameters.rail_volume_neg_ml", 500.0) * 1e-6);
  sp.leak_pos = (float)get_param_or<double>(this, "MPC_parameters.rail_leak_pos", 0.002);
  sp.leak_neg = (float)get_param_or<double>(this, "MPC_parameters.rail_leak_neg", 0.002);
  sp.pos_min  = 50.0f;  sp.pos_max = 800.0f;
  sp.neg_min  = 5.0f;   sp.neg_max = 110.0f;
  sp.pump     = refgen_ ? &refgen_->pump_table() : nullptr;   // 이미 만든 능력 테이블 재사용
  sp.finalize();

  mppi::SysMppiParams mp;
  mp.K        = get_param_or<int>(this, "MPC_parameters.sys_samples", 256);
  mp.NP       = (mpc_.mppi_np > 0) ? mpc_.mppi_np : mpc_.NP;
  mp.Ts       = (float)((mpc_.mppi_ts_s > 0.0) ? mpc_.mppi_ts_s : mpc_.Ts);
  mp.substeps = mpc_.mppi_substeps;
  mp.lambda   = (float)mpc_.mppi_lambda;
  mp.sigma_pct         = (float)mpc_.mppi_sigma_pct;
  mp.sigma_explore_pct = (float)mpc_.mppi_sigma_explore_pct;
  mp.explore_frac      = (float)mpc_.mppi_explore_frac;
  mp.noise_beta        = (float)mpc_.mppi_noise_beta;
  mp.w_track   = (float)((mpc_.mppi_w_track >= 0.0) ? mpc_.mppi_w_track : mpc_.Q_value);
  mp.w_effort  = (float)((mpc_.mppi_w_effort >= 0.0) ? mpc_.mppi_w_effort : mpc_.R_value);
  mp.w_du      = (float)mpc_.mppi_w_du;
  mp.w_rail    = (float)get_param_or<double>(this, "MPC_parameters.sys_w_rail", 0.5);
  mp.rail_scale_kpa = (float)get_param_or<double>(this, "MPC_parameters.sys_rail_scale_kpa", 20.0);
  mp.track_scale_kpa = (float)mpc_.mppi_track_scale_kpa;
  mp.terminal_mult   = (float)mpc_.mppi_terminal_mult;
  mp.du_limit_pct    = (float)mpc_.mppi_du_limit_pct;
  mp.rail_share  = (float)get_param_or<double>(this, "MPC_parameters.sys_rail_share", 0.20);
  mp.control_lines = get_param_or<bool>(this, "MPC_parameters.sys_control_lines", false);
  // 데드라인 [us]. 기본은 틱의 60%. 넘으면 다음 틱들을 건너뛰어 루프를 지킨다.
  sys_deadline_us_ = get_param_or<double>(this, "MPC_parameters.sys_deadline_us",
                                          0.6 * (double)period_ms_ * 1000.0);
  sys_control_lines_ = mp.control_lines;

  sys_mppi_ = std::make_unique<mppi::SystemSolver>(sp, mp, 0xC0FFEEu);
  sys_state_.resize(sp.n_ch);
  sys_uref_.assign((size_t)mppi::sys_nu(sp.n_ch), 0.0f);
  sys_exo_.V.assign((size_t)sp.n_ch, 1e-5f);
  sys_exo_.Vdot.assign((size_t)sp.n_ch, 0.0f);
  sys_exo_.P_ref.assign((size_t)sp.n_ch, (float)sensor_.kpa_atm());
  sys_exo_.u_macro.assign((size_t)sp.n_ch, 0.0f);

  RCLCPP_INFO(get_logger(),
    "MPC 솔버 = **중앙집중 MPPI**: 제어 %d개(채널 micro %d + atm %d + 라인 2), "
    "K=%d, NP=%d, Ts=%.1f ms (지평 %.0f ms), 서브스텝=%d, "
    "w=(track %.3g, rail %.3g, effort %.3g, du %.3g), 레일 기준 %.0f kPa, "
    "레일 부피 %.0f/%.0f mL, 펌프 테이블 %s",
    mppi::sys_nu(sp.n_ch), sp.n_ch, sp.n_ch, mp.K, mp.NP, mp.Ts * 1000.0,
    mp.NP * mp.Ts * 1000.0, mp.substeps,
    mp.w_track, mp.w_rail, mp.w_effort, mp.w_du, mp.rail_scale_kpa,
    sp.V_pos_m3 * 1e6, sp.V_neg_m3 * 1e6, sp.pump ? "재사용" : "없음(유량 0)");
}

// ============================================================================
// 레일 변화율 추정 — 채널별 MPPI 에 결합을 넣는 최소 장치
//
// 채널 12개의 **명목(피드포워드) 명령**으로 지금 이 순간의 레일 질량수지를 한 번 계산해
// dP_rail/dt 를 낸다. 롤아웃은 그것으로 레일을 선형 이동시킨다.
//
// 왜 이 형태인가: 전체를 하나의 MPPI 로 푸는 것도 구현해 봤는데(mppi_system), 공유
// 롤아웃에서는 채널 g 의 표본들이 자기 제어만 다른 게 아니라 **다른 11채널의 무작위
// 탐색이 만든 레일 궤적까지 달라서** 공로 배분이 뭉개졌다 (IAE 5.7 → 17.8).
// 명목만으로 레일을 예측하면 무작위성이 레일에 들어가지 않아 그 문제가 없다.
//
// 식은 sys_step 의 레일 항과 같다 (VirtualPowerpack::integrate 와 동일).
void Controller::estimate_rail_rates(double P_line_pos_kPa, double P_line_neg_kPa,
                                     double P_atm_kPa,
                                     float& dPpos_dt, float& dPneg_dt)
{
  dPpos_dt = dPneg_dt = 0.0f;
  if (sys_params_.ch.empty()) return;

  float draw_pos = 0.0f, fill_neg = 0.0f;
  for (auto& mpc : mpcs_) {
    AcadosMpc* m = mpc.get();
    const int gid = m->cfg().global_id;
    if (gid < 0 || gid >= (int)sys_params_.ch.size()) continue;
    const auto& cp = sys_params_.ch[(size_t)gid][mppi::V_MICRO];
    const float u_mi = m->uref()[0];              // 명목 micro 개도
    const float P    = m->p_used();
    const float z    = m->plant_est().v[0].z;
    if (m->cfg().is_positive)
      draw_pos += mppi::q_static(cp, u_mi, (float)P_line_pos_kPa, P, z);
    else
      fill_neg += mppi::q_static(cp, u_mi, P, (float)P_line_neg_kPa, z);
  }

  float Q_pump = 0.0f;
  if (sys_params_.pump)
    Q_pump = (float)(sys_params_.pump->flow_out(P_line_pos_kPa * 1000.0,
                                                P_line_neg_kPa * 1000.0)
                     / (double)mppi::LPM_TO_KGPS);

  const float u_vent  = (float)std::clamp(zoh_[pid_pos_pwm_index_] / 40.95, 0.0, 100.0);
  const float u_admit = (float)std::clamp(zoh_[pid_neg_pwm_index_] / 40.95, 0.0, 100.0);
  const float z_v = sys_state_.v[(size_t)sys_state_.iv_vent()].z;
  const float z_a = sys_state_.v[(size_t)sys_state_.iv_admit()].z;
  const float f_vent  = mppi::q_static(sys_params_.line, u_vent,
                                       (float)P_line_pos_kPa, (float)P_atm_kPa, z_v);
  const float f_admit = mppi::q_static(sys_params_.line, u_admit,
                                       (float)P_atm_kPa, (float)P_line_neg_kPa, z_a);
  const float lk_p = sys_params_.leak_pos * std::max(0.0f, (float)(P_line_pos_kPa - P_atm_kPa));
  const float lk_n = sys_params_.leak_neg * std::max(0.0f, (float)(P_atm_kPa - P_line_neg_kPa));

  const float K = mppi::RGAS_AIR * mppi::TEMP_K * mppi::LPM_TO_KGPS / 1000.0f;
  dPpos_dt = K * (Q_pump - f_vent - lk_p - draw_pos) / sys_params_.V_pos_m3;
  dPneg_dt = K * (f_admit + lk_n + fill_neg - Q_pump) / sys_params_.V_neg_m3;
}

void Controller::run_system_mppi(double P_atm_kPa, double P_line_pos_kPa,
                                 double P_line_neg_kPa, double P_line_macro_kPa,
                                 double P_line_macro_neg_kPa)
{
  const int n  = sys_params_.n_ch;

  // ── ① 채널별 prepare — **순차**. 채널당 수 µs 짜리 12개인데 풀 디스패치가
  //    그보다 비싸다 (틱당 풀 호출을 2회에서 1회로 줄인다).
  for (auto& mpc : mpcs_) {
    AcadosMpc* m = mpc.get();
    {
      const int brd_idx = m->cfg().can_board_id - 1;
      const int gid     = m->cfg().global_id;
      const bool pos    = m->cfg().is_positive;
      m->current_P_atm_       = (float)P_atm_kPa;
      m->current_P_now_       = (float)filt_out_[brd_idx];
      m->current_P_now_raw_   = (float)raw_out_[brd_idx];
      m->current_P_micro_     = (float)(pos ? P_line_pos_kPa : P_line_neg_kPa);
      m->current_P_macro_     = (float)P_line_macro_kPa;
      m->current_P_macro_neg_ = (float)P_line_macro_neg_kPa;
      m->set_volume((float)(final_active_vols_ml_[gid] * 1e-6));
      m->set_prev_volume((float)prev_vol_m3_[gid]);
      float ref_kpa = 0.f;
      if (gid >= 0 && gid < (int)ref_snapshot_.size()) ref_kpa = (float)ref_snapshot_[(size_t)gid];
      m->set_ref_value(ref_kpa);
      m->prepare((float)(dt_ctrl_sec_ * 1000.0), (float)elapsed_time_sec_);
    }
  }

  // ── ② 전체 시스템 상태·외생 입력·명목 조립 ────────────────────────────────
  sys_state_.P_pos = (float)P_line_pos_kPa;
  sys_state_.P_neg = (float)P_line_neg_kPa;
  sys_params_.P_macro     = (float)P_line_macro_kPa;
  sys_params_.P_macro_neg = (float)P_line_macro_neg_kPa;
  sys_params_.P_atm       = (float)P_atm_kPa;

  for (auto& mpc : mpcs_) {
    AcadosMpc* m = mpc.get();
    const int gid = m->cfg().global_id;
    if (gid < 0 || gid >= n) continue;
    const auto x0 = m->rollout_state();          // 채널 경로와 **같은 조립 규칙**
    sys_state_.P_ch[(size_t)gid] = x0.P;
    for (int j = 0; j < 3; ++j) sys_state_.v[(size_t)sys_state_.iv_ch(gid, j)] = x0.v[(size_t)j];
    sys_exo_.V[(size_t)gid]     = m->cfg().volume_m3;
    sys_exo_.Vdot[(size_t)gid]  = m->vol_dot_est();
    sys_exo_.P_ref[(size_t)gid] = m->cfg().ref_value;
    // macro 는 1단계에서 최적화 대상이 아니다 — 기존 게이트가 정한 피드포워드를 그대로 쓴다.
    sys_exo_.u_macro[(size_t)gid] = m->uref()[1];
    sys_uref_[(size_t)mppi::sys_i_micro(gid)]   = m->uref()[0];
    sys_uref_[(size_t)mppi::sys_i_atm(n, gid)]  = m->uref()[2];
  }
  sys_exo_.rail_pos_sp = (float)(gen_rail_pos_sp_kpa_ > 0.0 ? gen_rail_pos_sp_kpa_ : pid_pos_.ref);
  sys_exo_.rail_neg_sp = (float)(gen_rail_neg_sp_kpa_ > 0.0 ? gen_rail_neg_sp_kpa_ : pid_neg_.ref);
  sys_exo_.ref_tau_s   = (float)((mpc_.mppi_ref_tau_s > 0.0) ? mpc_.mppi_ref_tau_s
                                                             : mpc_.target_tc);

  // 라인 밸브 — PWM 값이 곧 개도다 (LinePID 의 (100−u) 반전은 그 안에서 끝난다).
  // 제어하지 않는 모드에서는 **외생 입력**으로만 넘겨 롤아웃이 레일을 예측하게 한다.
  const int iv = mppi::sys_i_vent(n), ia = mppi::sys_i_admit(n);
  const float u_vent_now  = (float)std::clamp(zoh_[pid_pos_pwm_index_] / 40.95, 0.0, 100.0);
  const float u_admit_now = (float)std::clamp(zoh_[pid_neg_pwm_index_] / 40.95, 0.0, 100.0);
  sys_exo_.u_vent  = u_vent_now;
  sys_exo_.u_admit = u_admit_now;
  if (sys_control_lines_) {
    sys_uref_[(size_t)iv] = u_vent_now;      // 명목 = 직전 인가값 (워밍 스타트)
    sys_uref_[(size_t)ia] = u_admit_now;
  }

  // ── ③ 한 번의 최적화. 표본을 스레드에 나눈다 (채널이 아니라) ──────────────
  // 표본을 **덩어리로** 나눈다. 표본 1개당 태스크 1개로 하면 태스크가 ~5 us 짜리인데
  // 디스패치·스핀대기 오버헤드가 그보다 커서 계산을 압도한다
  // (계측: K=256 개별 태스크 → 평균 1334 us, 최대 31205 us. 틱 예산 2 ms 의 15배).
  // 스레드 수만큼만 만들면 디스패치가 256 → 12 회로 줄고 표본별 결정론은 유지된다
  // (노이즈가 표본 인덱스로 시드되므로 어느 스레드가 어느 구간을 잡아도 같다).
  auto pfor = [this](int nn, const std::function<void(int)>& fn) {
    const int nthr = std::max(1, (int)pool_threads_);
    const int chunk = (nn + nthr - 1) / nthr;
    sys_tasks_.clear();
    for (int c = 0; c < nthr; ++c) {
      const int lo = c * chunk, hi = std::min(nn, lo + chunk);
      if (lo >= hi) break;
      sys_tasks_.emplace_back([&fn, lo, hi]() { for (int i = lo; i < hi; ++i) fn(i); });
    }
    pool_->run_batch_and_wait(sys_tasks_);
  };
  // ── 실시간 데드라인 ─────────────────────────────────────────────────────
  // 중앙집중 솔버는 평균은 예산 안이지만 산발적으로 틱 예산의 5~8배 스파이크가 난다.
  // 제어 루프가 멈추면 실기에서 위험하므로, 최근 실행이 예산을 넘겼으면 **이번 틱은
  // 건너뛰고 Δu=0(순수 피드포워드)** 을 쓴다. MPPI 는 워밍 스타트 기반이라 한 틱을
  // 쉬어도 다음 틱에 이어서 개선한다.
  static thread_local std::vector<float> zero_du;
  bool skipped = false;
  if (sys_over_budget_ > 0) {
    --sys_over_budget_;
    ++sys_skipped_;
    skipped = true;
  }
  const auto t_solve0 = std::chrono::steady_clock::now();
  const int nu_sys = mppi::sys_nu(n);
  if (zero_du.size() != (size_t)nu_sys) zero_du.assign((size_t)nu_sys, 0.0f);
  const auto& du = skipped ? zero_du
                           : sys_mppi_->solve(sys_state_, sys_exo_, sys_uref_, pfor);
  if (!skipped) {
    const double us = std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - t_solve0).count();
    // 예산 초과 시 다음 N 틱을 쉬게 해서 루프가 회복할 시간을 준다.
    if (us > sys_deadline_us_) {
      sys_over_budget_ = std::max(1, (int)(us / std::max(1.0, (double)period_ms_ * 1000.0)));
      ++sys_over_cnt_;
    }
  }

  // ── ④ 채널별 finish (테이퍼·클램프·PWM·상태 추정) ─────────────────────────
  for (auto& mpc : mpcs_) {
    AcadosMpc* m = mpc.get();
    const int gid = m->cfg().global_id;
    if (gid < 0 || gid >= n) continue;
    // macro Δu = 0 — 1단계에서는 최적화 대상이 아니다.
    const std::array<float,3> du3{ du[(size_t)mppi::sys_i_micro(gid)], 0.0f,
                                   du[(size_t)mppi::sys_i_atm(n, gid)] };
    std::array<uint16_t, MPC_OUT_DIM> u3{};
    m->finish(du3, u3);
    const int base = (m->cfg().can_board_id - 1) * PWM_PER_BOARD;
    zoh_[base + 0] = u3[0];
    zoh_[base + 1] = u3[1];
    zoh_[base + 2] = u3[2];
  }

  // ── ⑤ 라인 밸브 ──────────────────────────────────────────────────────────
  const float u_vent  = sys_control_lines_
      ? std::clamp(sys_uref_[(size_t)iv] + du[(size_t)iv], 0.0f, 100.0f) : u_vent_now;
  const float u_admit = sys_control_lines_
      ? std::clamp(sys_uref_[(size_t)ia] + du[(size_t)ia], 0.0f, 100.0f) : u_admit_now;
  if (sys_control_lines_) {
    // 비유한 값 차단 — 라인 밸브는 레일 전체를 좌우하므로 특히 위험하다.
    const float uv = std::isfinite(u_vent)  ? std::clamp(u_vent,  0.0f, 100.0f) : 0.0f;
    const float ua = std::isfinite(u_admit) ? std::clamp(u_admit, 0.0f, 100.0f) : 0.0f;
    zoh_[pid_pos_pwm_index_] = (uint16_t)std::lround(uv * 40.95f);
    zoh_[pid_neg_pwm_index_] = (uint16_t)std::lround(ua * 40.95f);
  }

  // 라인 밸브 내부 상태를 실제 인가 명령으로 전진 (채널은 finish 가 한다)
  {
    const float dt = (float)dt_ctrl_sec_;
    auto adv = [&](int idx, float u, float pin, float pout) {
      auto& vs = sys_state_.v[(size_t)idx];
      const float z = mppi::step_bw(sys_params_.line, vs, u);
      mppi::valve_dyn(sys_params_.line, vs,
                      mppi::q_static(sys_params_.line, u, pin, pout, z), dt);
    };
    adv(sys_state_.iv_vent(),  u_vent,  (float)P_line_pos_kPa, (float)P_atm_kPa);
    adv(sys_state_.iv_admit(), u_admit, (float)P_atm_kPa,      (float)P_line_neg_kPa);
  }

  // ── ⑥ 진단 — 예측 레일압 vs 측정 레일압 ───────────────────────────────────
  // 이것이 VirtualPowerpack 과 모델이 어긋났는지 보는 **연속 교차검증**이다.
  // 두 파일의 식이 갈리면 이 잔차가 커진다.
  // **직전 틱의 1스텝 예측**을 이번 측정과 비교한다. 40 ms 앞 예측을 현재 측정과
  // 비교하면 당연히 크게 벌어진다 (그 실수로 22.8 kPa 로 보였다).
  if (sys_pred1_valid_) {
    sys_pred_err_pos_ += std::abs(sys_pred1_pos_ - P_line_pos_kPa);
    sys_pred_err_neg_ += std::abs(sys_pred1_neg_ - P_line_neg_kPa);
    ++sys_pred_n_;
  }
  sys_pred1_pos_ = (double)sys_mppi_->pred1_rail_pos();
  sys_pred1_neg_ = (double)sys_mppi_->pred1_rail_neg();
  sys_pred1_valid_ = sys_mppi_->pred1_valid();
  if (++sys_stat_tick_ >= 5000) {
    sys_stat_tick_ = 0;
    const auto st = sys_mppi_->take_stats();
    if (st.calls) {
          RCLCPP_INFO(get_logger(),
        "중앙집중 MPPI: %.0f us 평균 / %.0f us 최대 (틱 %d ms), 유효샘플 %.1f/%d, "
        "Jmin %.4f, 초과 %.4f, 첫스텝 포화 %.1f%%, 평평 %.1f%% | "
        "레일 예측오차 양 %.2f / 음 %.2f kPa",
        st.sum_us / (double)st.calls, (double)st.max_us, period_ms_,
        st.sum_eff / (double)st.calls, sys_mppi_->params().K, st.sum_cost / (double)st.calls,
        st.sum_spread / (double)st.calls,
        100.0 * (double)st.sat_first / (double)(st.calls * mppi::sys_nu(n)),
        100.0 * (double)st.flat / (double)st.calls,
        sys_pred_err_pos_ / std::max(1, sys_pred_n_),
        sys_pred_err_neg_ / std::max(1, sys_pred_n_));
      if (sys_over_cnt_ > 0 || sys_skipped_ > 0)
        RCLCPP_WARN(get_logger(),
          "중앙집중 MPPI 데드라인: %ld회 초과(%.0f us 기준), %ld틱 건너뜀 — "
          "sys_samples 를 줄이거나 period_ms 를 올릴 것",
          (long)sys_over_cnt_, sys_deadline_us_, (long)sys_skipped_);
      sys_over_cnt_ = sys_skipped_ = 0;
    }
    sys_pred_err_pos_ = sys_pred_err_neg_ = 0.0; sys_pred_n_ = 0;
  }
}

void Controller::on_timer() {
  // 경과 시간을 **틱 카운트**에서 만든다. 벽시계로 만들면 5초 안에 제어 틱이 몇 번
  // 들어가는지가 실행마다 달라져, 아래 두 게이트(밸브 잠금 해제 / 적분 리셋 해제)가
  // 서로 다른 시점에 풀린다 — 같은 빌드 반복 실행에서 정상상태 밸브 개방률이
  // 0/32/100% 로 흩어진 원인 중 하나다 (README 0절).
  // 틱 기준이면 같은 입력에 항상 같은 궤적이 나오고, 오프라인 하네스로 옮길 때도
  // 시간 주입점이 여기 한 곳으로 끝난다.
  elapsed_time_sec_ = (double)tick_ * (double)std::max(1, period_ms_) / 1000.0;
  const auto tick_now = std::chrono::steady_clock::now();
  wall_elapsed_sec_ = std::chrono::duration<double>(tick_now - start_time_).count();

  // ── 이번 틱의 dt ────────────────────────────────────────────────────
  // elapsed_time_sec_ 는 게이트 재현성을 위해 **틱 기반**으로 남기고(README 0절),
  // 물리 계산에 쓰는 dt 만 실측으로 바꾼다. 둘의 역할이 다르다:
  // 전자는 "몇 번째 틱인가", 후자는 "얼마나 시간이 흘렀는가" 다.
  {
    const double nom = (double)std::max(1, period_ms_) / 1000.0;
    if (tick_ > 0) {
      const double raw = std::chrono::duration<double>(tick_now - last_tick_time_).count();
      // 스케줄러 스파이크가 dt 를 오염시키지 않게 공칭의 [0.25, 4] 배로 자른다.
      const double d = std::clamp(raw, 0.25 * nom, 4.0 * nom);
      dt_meas_sec_ = (dt_meas_sec_ <= 0.0) ? d : (0.02 * d + 0.98 * dt_meas_sec_);
    }
    last_tick_time_ = tick_now;
    dt_ctrl_sec_ = (use_measured_dt_ && dt_meas_sec_ > 0.0) ? dt_meas_sec_ : nom;
  }

  std::array<uint16_t, NUM_CAN_BOARDS> snap_sensors;
  {
    std::lock_guard<std::mutex> lk(sensors_mtx_);
    snap_sensors = sensors_raw_;
  }

  // ----------------------------------------------------------------
  // 0-. **센서가 유효해지기 전에는 밸브를 절대 건드리지 않는다**
  //
  // raw 0 = 그 보드 프레임을 한 번도 못 받았다는 뜻이다. 예전에는 그걸 그대로
  // 환산해 (0 − 1112) × 0.25 + 101.325 = **−176.7 kPa** 를 진짜 압력으로 믿었다.
  // 컨트롤러는 "챔버가 대기압보다 278 kPa 낮다" 로 보고 micro·macro 를 활짝 열고,
  // 그 지령이 CAN 송신 큐에 쌓였다. CAN 이 8.5 초 뒤 살아나자 backlog 가 순서대로
  // 쏟아져 탱크 580 kPa 가 챔버로 들어갔고(228 kPa) 팔이 스토퍼를 넘었다 —
  // 20260829_201659, 액추에이터 파손.
  //
  // 0 kPa 로 붙잡아 두는 것도 안 된다 (그것도 진공이라 똑같이 채우려 든다).
  // 필요한 보드(라인 4 개 + 활성 채널) 가 **하나라도** 아직 안 왔으면 제어를
  // 시작하지 않는다. 여기서 return 하면 PWM 이 안 나가고, 브리지의 PWM 워치독이
  // 200 ms 뒤 안전 상태(채널 폐쇄 + 라인 전개)로 간다.
  // ----------------------------------------------------------------
  {
    std::string bad;
    for (int bid = 1; bid <= 16; ++bid) {
      const int idx = bid - 1;
      if (snap_sensors[idx] > 0) sensor_seen_[(size_t)idx] = true;
      if (sensor_seen_[(size_t)idx]) continue;
      const int gid = bid - channel_board_offset_;
      const bool is_line = (bid == P_pos_board_id_ || bid == P_neg_board_id_ ||
                            bid == P_macro_board_id_ || bid == P_macro_neg_board_id_);
      const bool needed = is_line ||
          (gid >= 0 && gid < num_total_channels_ && active_channels_.count(gid) > 0);
      if (needed) bad += (bad.empty() ? "" : ", ") + std::to_string(bid);
    }
    if (!bad.empty()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
        "센서 대기: board %s 의 프레임이 **한 번도** 안 왔다 (raw=0). 제어를 "
        "시작하지 않는다 — 이 상태로 돌면 압력을 −176.7 kPa 로 읽고 밸브를 활짝 "
        "연다 (20260829 액추에이터 파손 경로). CAN 배선·보드 전원을 확인할 것.",
        bad.c_str());
      return;
    }
  }

  // ----------------------------------------------------------------
  // 0. Sensor zero-calibration: boards 1~16 초기화 (0.5초간 평균)
  // ----------------------------------------------------------------
  if (!sensor_zeroed_) {
    for (int i = 0; i < 16; ++i) {
      if (snap_sensors[i] > 0) {
        sensor_zero_sum_[i] += snap_sensors[i];
        sensor_zero_cnt_[i]++;
      }
    }
    sensor_zero_tick_++;

    if (sensor_zero_tick_ >= ZERO_SAMPLES) {
      for (int i = 0; i < 16; ++i) {
        if (sensor_zero_cnt_[i] == 0) continue;
        const double meas = sensor_zero_sum_[i] / sensor_zero_cnt_[i];
        // 0점 보정은 "지금 이 압력이 대기압이다" 라고 선언하는 것이다. 계통에 잔압이
        // 남은 채 부르면 그 채널의 영점이 잔압만큼 통째로 밀리고, **과압 세이프티도
        // 같이 밀린다** (190 kPa 트립이 실제 210 kPa 에서 걸리는 식). 조용히 넘어가면
        // 안 되므로 yaml 값과의 차이를 kPa 로 환산해 경고한다.
        const double dev_kpa = (meas - yaml_offset_[(size_t)i]) * sensor_.boards[i].gain;
        if (std::abs(dev_kpa) > zero_tolerance_kpa_)
          RCLCPP_WARN(get_logger(),
            "0점 보정: board %d 가 yaml 기준에서 %+.1f kPa 벗어났다 "
            "(offset %.1f → %.1f). 계통에 잔압이 남아 있지 않은지 확인할 것 — "
            "영점이 밀리면 과압 세이프티도 같이 밀린다.",
            i + 1, dev_kpa, yaml_offset_[(size_t)i], meas);
        sensor_.boards[i].offset = meas;
      }
      sensor_zeroed_ = true;
      RCLCPP_INFO(get_logger(), "=== Sensor zero-calibration complete ===");
      for (int bid = 1; bid <= 16; ++bid)
        RCLCPP_INFO(get_logger(), "  Board %2d: offset=%.1f", bid, sensor_.boards[bid-1].offset);
    }
    return;   // 초기화 중에는 제어 출력 없음
  }

  // ----------------------------------------------------------------
  // 1. Raw Data -> kPa 변환 및 LPF 적용 (보드별 flat 루프)
  // ----------------------------------------------------------------
  for (int bid = 1; bid <= NUM_CAN_BOARDS; ++bid) {
    int idx = bid - 1;

    // 엔코더 보드 (17~22): **여기서 각도를 만들지 않는다.**
    //
    // board/sensors 는 보드 1~16 만 싣는다 (CanBridge 가 PWM_BOARDS=16 개만 낸다).
    // 그래서 snap_sensors[16..21] 은 언제나 0 이고, 그걸 반전증폭 역산에 넣으면
    // (4125−0)/0.825 = 5000 mV 라 보정에 따라 −85° 니 +247° 니 하는 상수가 나온다.
    // 실제 각도는 CanBridge 가 board/analog 로 내고 encoder_angles_ 로 들어온다.
    //
    // 이 죽은 경로 때문에 Sensor_calibration 에 엔코더 보정 사본을 두게 됐고,
    // 그 사본이 EncoderCalibration 과 어긋나 두 번 사고를 냈다 (S-28 board 17,
    // S-30 board 18). 사본은 지웠고 여기서는 0 을 낸다 — controller/sensors_kpa
    // 를 보는 쪽에 "이 슬롯엔 압력이 없다" 가 분명해진다.
    if (bid >= 17 && bid <= 22) {
      filt_out_[idx] = 0.0;
      continue;
    }

    // 라인 압력 보드 or 활성 채널 보드만 처리
    bool is_line_board = (bid == P_pos_board_id_ || bid == P_neg_board_id_ || bid == P_macro_board_id_ || bid == P_macro_neg_board_id_);
    int gid = bid - channel_board_offset_;
    if (!is_line_board && (gid < 0 || gid >= num_total_channels_ || active_channels_.count(gid) == 0)) {
      filt_out_[idx] = sensor_.kpa_atm();
      raw_out_[idx]  = sensor_.kpa_atm();
      continue;
    }
    // raw 0 = 그 보드 프레임을 **못 받았다** (CAN 두절·보드 전원). 이걸 그대로
    // 환산하면 (0 − 1112) × 0.25 + 101.325 = **−176.7 kPa** 라는 물리적으로
    // 불가능한 값이 나오고, 컨트롤러는 "대기압보다 278 kPa 낮다" 로 믿어 밸브를
    // 활짝 연다. 20260829_201659 의 액추에이터 파손이 정확히 그 경로였다.
    // 직전 값을 유지하고 소리를 낸다 (브리지의 RX 워치독가 별도로 안전 상태로 간다).
    if (snap_sensors[idx] == 0) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
        "board %d raw=0 — 프레임이 없다. 압력을 직전 값(%.1f kPa)으로 유지한다. "
        "그대로 환산하면 −176.7 kPa 가 되어 밸브가 활짝 열린다.", bid, filt_out_[idx]);
      continue;
    }
    double raw_kpa = sensor_.kpa(bid, snap_sensors[idx]);
    if (!filter_initialized_) filt_state_[idx] = raw_kpa;
    filt_state_[idx] = sensor_filter_alpha_ * raw_kpa + (1.0 - sensor_filter_alpha_) * filt_state_[idx];
    filt_out_[idx]   = filt_state_[idx];
    raw_out_[idx]    = raw_kpa;              // 예측기 초기 상태 비교용 (지연 없음, 잡음 있음)
  }
  filter_initialized_ = true;

  // 보강 설정을 각 MPC 에 반영 (ros2 param set 이 바꾼 값이 다음 틱부터 적용된다)
  push_aug_to_mpcs();

  // 보강 상태 진단 — 어느 항이 얼마나 일하고 있는지 보여야 켜고 끌 판단이 선다.
  if (tick_ % 1000 == 0 && !mpcs_.empty()) {
    ControlAug snap; { std::lock_guard<std::mutex> lk(aug_mtx_); snap = aug_; }
    if (snap.adapt_gain || snap.offset_free || snap.auto_tune) {
      std::string line;
      for (auto& m : mpcs_) {
        char buf[96];
        snprintf(buf, sizeof(buf), " ch%d[k=%.2f d=%+.1f tau=%.3f]",
                 m->cfg().global_id, m->k_flow(), m->d_hat(), m->tau_used());
        line += buf;
      }
      RCLCPP_INFO(get_logger(), "[보강 %s%s%s]%s",
                  snap.adapt_gain ? "이득 " : "", snap.offset_free ? "오프셋 " : "",
                  snap.auto_tune ? "튜닝 " : "", line.c_str());
    }
  }

  // ----------------------------------------------------------------
  // 2. 필터링된 값을 Topic으로 Publish
  // ----------------------------------------------------------------
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(filt_out_.begin(), filt_out_.end());
    pub_kpa_all_->publish(msg);
  }

  const double P_line_pos_kPa       = filt_out_[P_pos_board_id_ - 1];
  const double P_line_neg_kPa       = filt_out_[P_neg_board_id_ - 1];
  const double P_line_macro_kPa     = filt_out_[P_macro_board_id_ - 1];
  const double P_line_macro_neg_kPa = filt_out_[P_macro_neg_board_id_ - 1];
  const double P_atm_kPa            = sensor_.kpa_atm();

  // ----------------------------------------------------------------
  // 위치 제어: filt_out_ 갱신 이후, ref_snapshot 이전에 실행
  // 각도 → 압력 레퍼런스 변환 후 mpc_ref_kpa_ 에 기록
  // ----------------------------------------------------------------
  slew_targets(std::max(1e-6, dt_ctrl_sec_));
  if (control_mode_ == 1) {
    run_position_control(std::max(1e-6, dt_ctrl_sec_));
  } else if (control_mode_ == 2) {
    run_optimized_pressure_ref(std::max(1e-6, dt_ctrl_sec_));
  }

  {
    std::lock_guard<std::mutex> lk(mpc_ref_mtx_);
    ref_snapshot_ = mpc_ref_kpa_;
  }
  if (pub_mpc_refs_) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(ref_snapshot_.begin(), ref_snapshot_.end());
    pub_mpc_refs_->publish(msg);
  }

  // ----------------------------------------------------------------
  // 액추에이터 연결 시 엔코더 각도로 부피 계산 (actuator_connected=true)
  //
  //   A     = π × (piston_dia/2)²  = π × 25²  [mm²]   (Ø50 mm 피스톤)
  //   reel  = 25 mm  — 조인트에서 피스톤 로드까지의 릴 반경
  //
  //   피스톤 변위 x = reel × θ  이고 θ 는 **라디안**이다. 엔코더는 도(deg)로 오므로
  //   반드시 변환해야 한다. (도를 그대로 넣으면 45°에서 V⁺가 2338 mL 이 되어
  //   Ø50 mm 실린더로 불가능한 값이 나온다 — 원래 있던 단위 버그.)
  //
  //   양압: V_pos = tank_pos_mL + A × max(0,  40 + reel×θ_rad) / 1000  [mL]
  //   음압: V_neg = tank_neg_mL + A × max(0,  90 - reel×θ_rad) / 1000  [mL]
  //
  //   actuator i  →  encoder board (17+i)  →  ang[i]  [deg]
  // ----------------------------------------------------------------
  if (actuator_connected_) {
      std::array<double, 9> ang;
      {
          std::lock_guard<std::mutex> lk(sensors_mtx_);
          ang = encoder_angles_;
      }
      const double A = piston_area_mm2_;
      for (int i = 0; i < num_positive_channels_; ++i) {
          if (active_channels_.count(i) == 0) continue;
          const double x_mm    = reel_radius_mm_ * ang[i] * M_PI / 180.0;   // 피스톤 변위
          const int    neg_gid = num_positive_channels_ + i;

          // 기하 모델에 **채널별 배율**을 곱한다.
          //
          // 절대 오버라이드(channel_config.chN.volume_ml)는 여기서 매 틱 덮어써져
          // 소용이 없다 — 액추에이터가 붙어 있으면 부피가 각도의 함수이기 때문이다.
          // 배율이면 각도 의존성은 그대로 두고 크기만 고친다.
          //
          // 실기 20260829_195910: 세 축이 같은 게인·질량·기하인데 axis1 만 진동했다
          // (각도 p-p: ax0 4.6° / ax2 3.2° / **ax1 22.0°**). 같은 지령·차압에서
          // dP/dt 를 재니 ch1 이 ch0 의 4.97 배였다. dP/dt = ṁ·R·T/V 이므로
          // 유효 부피가 1/5 라는 뜻이고, 기하 모델이 146 mL 로 보는 동안 MPC 는
          // 5 배 유량을 요구해 과개방하고 있었다.
          vol_ml_[i] = vol_scale_[(size_t)i] * (tank_volume_pos_ml_
              + A * std::max(0.0, vol_offset_pos_mm_ + x_mm) / 1000.0);

          if (neg_gid < num_total_channels_ && active_channels_.count(neg_gid))
              vol_ml_[neg_gid] = vol_scale_[(size_t)neg_gid] * (tank_volume_neg_ml_
                  + A * std::max(0.0, vol_offset_neg_mm_ - x_mm) / 1000.0);
      }
  }

  std::fill(final_active_vols_ml_.begin(), final_active_vols_ml_.end(), 0.0);
  for(int i = 0; i < num_total_channels_; ++i) {
      if (active_channels_.count(i) == 0) continue;
      final_active_vols_ml_[i] = vol_ml_[i];
  }

  if (pub_active_vols_) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(final_active_vols_ml_.begin(), final_active_vols_ml_.end());
    pub_active_vols_->publish(msg);
  }

  // ── 중앙집중 MPPI 경로 ───────────────────────────────────────────────────
  // 12채널을 각자 풀지 않고 라인 밸브 2개까지 포함해 **한 번에** 푼다.
  // 여기서 return 하지 않고 아래 공통 후처리(macro 스위치·과압 보호·PWM 발행)로 간다.
  if (sys_mppi_) {
    run_system_mppi(P_atm_kPa, P_line_pos_kPa, P_line_neg_kPa,
                    P_line_macro_kPa, P_line_macro_neg_kPa);
  } else {

  // 레일 변화율을 **한 번** 계산해 12채널이 공유한다 (명목 기반이라 무작위성이 없다).
  if (rail_rate_enable_ && !sys_params_.ch.empty())
    estimate_rail_rates(P_line_pos_kPa, P_line_neg_kPa, P_atm_kPa,
                        rail_rate_pos_, rail_rate_neg_);
  else
    rail_rate_pos_ = rail_rate_neg_ = 0.0f;

  const int phase = static_cast<int>(tick_ % MPC_PHASES);
  std::vector<std::function<void()>> tasks;
  
  // MPC Task 생성 (필터된 압력값 캡처)
  for (auto& mpc : mpcs_) {
    if ((mpc->cfg().global_id % MPC_PHASES) != phase) continue;

    AcadosMpc* m = mpc.get();  // capture raw pointer so each lambda binds its own MPC
    tasks.emplace_back([this,
                        m,
                        P_line_pos_kPa, P_line_neg_kPa, P_line_macro_kPa, P_line_macro_neg_kPa, P_atm_kPa]() {
      const int brd_idx = m->cfg().can_board_id - 1;
      const double P_state_kPa = filt_out_[brd_idx];
      const bool pos_side = m->cfg().is_positive;

      m->current_P_atm_       = static_cast<float>(P_atm_kPa);
      m->current_P_now_       = static_cast<float>(P_state_kPa);
      m->current_P_now_raw_   = static_cast<float>(raw_out_[brd_idx]);
      m->current_P_micro_     = static_cast<float>(pos_side ? P_line_pos_kPa : P_line_neg_kPa);
      m->current_P_macro_     = static_cast<float>(P_line_macro_kPa);
      m->current_P_macro_neg_ = static_cast<float>(P_line_macro_neg_kPa);

      const int gid = m->cfg().global_id;

      m->set_volume(static_cast<float>(final_active_vols_ml_[gid] * 1e-6));
      m->set_prev_volume(static_cast<float>(prev_vol_m3_[gid]));

      float ref_kpa = 0.f;
      if (gid >= 0 && gid < (int)ref_snapshot_.size()) ref_kpa = (float)ref_snapshot_[(size_t)gid];
      m->set_ref_value(ref_kpa);

      if (gid == log_channel_id_ && log_file_.is_open()) {
        log_file_ << tick_ << "," << ref_kpa << "," << P_state_kPa << "\n";
      }

      m->set_rail_rate(pos_side ? rail_rate_pos_ : rail_rate_neg_);
      std::array<uint16_t, MPC_OUT_DIM> u3{};
      m->solve(static_cast<float>(dt_ctrl_sec_ * 1000.0), u3, static_cast<float>(elapsed_time_sec_));

      const int pwm_base = brd_idx * PWM_PER_BOARD;
      zoh_[pwm_base + 0] = u3[0];
      zoh_[pwm_base + 1] = u3[1];
      zoh_[pwm_base + 2] = u3[2];
    });
  }

  pool_->run_batch_and_wait(tasks);
  }   // ← 채널별 경로 끝 (중앙집중 경로는 위에서 이미 zoh_ 를 채웠다)

  if (macro_switch_pwm_index_ >= 0 && macro_switch_pwm_index_ < PWM_TOTAL) {
    bool any_neg_macro_active = false;
    for (int gid = num_positive_channels_; gid < num_total_channels_; ++gid) {
      int pwm_macro_idx = (gid + channel_board_offset_ - 1) * PWM_PER_BOARD + 2;  // v3 = macro
      if (pwm_macro_idx < PWM_TOTAL && zoh_[pwm_macro_idx] > 0) {
        any_neg_macro_active = true;
        break;
      }
    }
    zoh_[(size_t)macro_switch_pwm_index_] = any_neg_macro_active ? 4095 : 0;
  }

  const double dt = std::max(1e-6, dt_ctrl_sec_);   // LinePID

  // 중앙집중 MPPI 는 라인 밸브 2개를 **직접** 지령한다 (라인 PID 를 흡수).
  // 그 경로에서 PID 를 함께 돌리면 같은 PWM 인덱스에 두 주인이 생긴다.
  //
  // 흡수가 원리적으로 옳은 이유: 양압 PID 의 액추에이터는 **대기 방출 밸브뿐**이라
  // 레일압을 올릴 수단이 없고 정상상태에 이미 포화(u=100)해 있다. 채널들이 레일을
  // 끌어내릴 때 남은 권한이 0 이다. 반면 MPPI 는 "지금 채널들이 얼마를 빼갈 것인가"를
  // 지평 안에서 알고 있으므로 미리 덜 버리거나 채널 개도를 조절할 수 있다.
  // 중앙집중이라도 라인 밸브를 MPPI 가 소유하지 않으면 LinePID 는 그대로 돌아야 한다
  // (레일은 예측만 하고 제어는 맡긴다 — 계측상 MPPI 가 라인을 잡으면 불안정했다).
  const bool line_pid_active = (sys_mppi_ == nullptr) || !sys_control_lines_;

  // -------------------------------------------------------------
  // [양압 라인 PID] (Positive Line)
  // -------------------------------------------------------------
  if (line_pid_active) {
    const double err = pid_pos_.ref - P_line_pos_kPa;
    
    pid_pos_state_.integ += err * dt;
    double deriv = 0.0;
    if (pid_pos_state_.has_prev) deriv = (err - pid_pos_state_.prev_err) / dt;

    double u = pid_pos_.kp * err + pid_pos_.ki * pid_pos_state_.integ + pid_pos_.kd * deriv;
    
    // Anti-windup
    double u_clamped = std::clamp(u, pid_out_min_, pid_out_max_);
    if (u != u_clamped) {
      double excess = u - u_clamped;
      if (std::abs(pid_pos_.ki) > 1e-6) pid_pos_state_.integ -= excess / pid_pos_.ki;
      u = u_clamped;
    } else {
      u = u_clamped;
    }

    pid_pos_state_.prev_err = err;
    pid_pos_state_.has_prev = true;

    const double inverted_u = pid_out_max_ - u;
    // [수정] 4095 스케일 (40.95 = 4095/100)
    const uint16_t pwm = static_cast<uint16_t>( std::round(inverted_u * 40.95) );
    
    if (pid_pos_pwm_index_ >= 0 && pid_pos_pwm_index_ < PWM_TOTAL) {
      zoh_[(size_t)pid_pos_pwm_index_] = pwm;
    }
  }

  // -------------------------------------------------------------
  // [음압 라인 PID] (Negative Line)
  // -------------------------------------------------------------
  if (line_pid_active) {
    const double err = P_line_neg_kPa - pid_neg_.ref; 
    
    pid_neg_state_.integ += err * dt;
    double deriv = 0.0;
    if (pid_neg_state_.has_prev) deriv = (err - pid_neg_state_.prev_err) / dt;

    double u = pid_neg_.kp * err + pid_neg_.ki * pid_neg_state_.integ + pid_neg_.kd * deriv;
    
    double u_clamped = std::clamp(u, pid_out_min_, pid_out_max_);
    if (u != u_clamped) {
      double excess = u - u_clamped;
      if (std::abs(pid_neg_.ki) > 1e-6) pid_neg_state_.integ -= excess / pid_neg_.ki;
    }
    u = u_clamped;

    pid_neg_state_.prev_err = err;
    pid_neg_state_.has_prev = true;

    const double inverted_u = pid_out_max_ - u;
    // [수정] 4095 스케일 (40.95 = 4095/100)
    const uint16_t pwm = static_cast<uint16_t>( std::round(inverted_u * 40.95) );

    if (pid_neg_pwm_index_ >= 0 && pid_neg_pwm_index_ < PWM_TOTAL) {
      zoh_[(size_t)pid_neg_pwm_index_] = pwm;
    }
  }

  inner_loop_1khz(static_cast<float>(period_ms_));

  if (tick_ > 0 && (tick_ % 5000) == 0) {
    const double ratio = wall_elapsed_sec_ / std::max(1e-9, elapsed_time_sec_);
    if (ratio < 0.97 || ratio > 1.03)
      RCLCPP_WARN(get_logger(),
        "틱 간격: 가정 %.1f ms, 실측 평균 %.2f ms (비 %.3f). dt 는 %s. "
        "period_ms 를 실측에 맞추면 게이트 시간(elapsed)도 함께 맞는다.",
        (double)period_ms_, (double)period_ms_ * ratio, ratio,
        use_measured_dt_ ? "실측값을 쓴다 (모델 오차 없음)"
                         : "period_ms 를 쓴다 — 이 괴리가 그대로 모델 오차다");
  }

  if (sys_valve_operate_ && elapsed_time_sec_ >= 5.0) {
    for (int i = 0; i < PWM_TOTAL; ++i)
      cmds_[i] = clamp_pwm(static_cast<int>(zoh_[i]) + inner_[i]);
  } else {
    std::fill(cmds_.begin(), cmds_.end(), 0);
  }

  // ----------------------------------------------------------------
  // Over-pressure safety: positive channels (hysteresis latch)
  // Latch ON  when P >= limit            → force exhaust fully open
  // Latch OFF when P <  limit - hyst     → return control to MPC
  // ----------------------------------------------------------------
  for (int gid = 0; gid < num_positive_channels_; ++gid) {
    if (active_channels_.count(gid) == 0) continue;
    int bid     = gid + channel_board_offset_;
    int brd_idx = bid - 1;
    if (brd_idx < 0 || brd_idx >= NUM_CAN_BOARDS) continue;

    const double P = filt_out_[brd_idx];
    const double release_threshold = pressure_safety_limit_kpa_ - pressure_safety_hysteresis_kpa_;

    if (P >= pressure_safety_limit_kpa_) {
      safety_latched_[gid] = true;
    } else if (P < release_threshold) {
      safety_latched_[gid] = false;
    }

    if (auto* m = mpc_for_gid(gid)) m->set_safety_latched(safety_latched_[gid]);
    if (safety_latched_[gid]) {
      int base        = brd_idx * PWM_PER_BOARD;
      cmds_[base + 0] = 0;     // micro valve: closed
      cmds_[base + 1] = 4095;  // exhaust valve: fully open
      cmds_[base + 2] = 0;     // macro valve: closed
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
        "[SAFETY] ch%d (board %d) P=%.1f kPa, latched (limit=%.1f, release=%.1f)",
        gid, bid, P, pressure_safety_limit_kpa_, release_threshold);
    }
  }

  publish_cmds();

  for(int i = 0; i < num_total_channels_; ++i) {
      prev_vol_m3_[i] = final_active_vols_ml_[i] * 1.0e-6;
  }

  ++tick_;
}

// ================================
// run_position_control
// ================================
// 외층 위치 PID + 중력 피드포워드 + 마찰 보상
// 결과를 mpc_ref_kpa_ 에 기록하면 이후 MPC 내층이 압력 추종
//
// 중력 토크: τ = m·g·L·cos(90°-θ) = m·g·L·sin(θ_rad)
// P_ff = kff_gravity × τ          [kPa]
//
// 마찰 (쿨롱):
//   운동 중 → sign(vel) × friction_kpa   (운동 방향 반대로 보상)
//   정지 근처 → sign(error) × friction_kpa (정지마찰 극복)
//
// 출력:
//   P_pos_ref = p_bias_pos + (P_pid + P_ff + P_friction)
//   P_neg_ref = p_bias_neg - (P_pid + P_ff + P_friction) × neg_coupling
//
// actuator_connected_=false (액추에이터 미연결, 순수 압력추종 테스트):
//   엔코더 각도가 고정돼 있어 PID/마찰 보상은 의미가 없으므로 끄고,
//   중력 FF만 목표각(angle_ref) 기준으로 계산해 목표압력을 만든다.
//   → position_ref_client.py로 보낸 각도(30°, 45°, ...)마다 서로 다른
//     목표압력이 생성되고, 그 압력을 MPC(단일 채널)가 추종하는지 확인 가능.
void Controller::run_position_control(double dt_sec)
{
  const int n = (int)pos_ctrl_cfg_.size();
  std::vector<double> dbg_all;
  dbg_all.reserve((size_t)n * 8);

  for (int a = 0; a < n; ++a) {
    auto& cfg   = pos_ctrl_cfg_[(size_t)a];
    auto& state = pos_ctrl_state_[(size_t)a];

    // 엔코더 각도: board/analog 토픽 → encoder_angles_[] (sensors_mtx_ 보호)
    // board/sensors raw(filt_out_) 는 데이터가 없으면 0이라 사용 불가
    const int enc_idx = cfg.actuator_idx;
    if (enc_idx < 0 || enc_idx >= (int)encoder_angles_.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[PosCtrl axis%d] actuator_idx=%d out of range", a, enc_idx);
      dbg_all.insert(dbg_all.end(), 8, 0.0);
      continue;
    }
    double angle;
    {
      std::lock_guard<std::mutex> lk(sensors_mtx_);
      angle = encoder_angles_[(size_t)enc_idx];
    }

    // 목표 각도: TCP 수신 전까지는 현재 각도 유지 (급격한 움직임 방지)
    double angle_ref;
    {
      std::lock_guard<std::mutex> lk(mpc_ref_mtx_);
      // 항상 슬루된 목표를 쓴다. 예전에는 TCP 수신 전에 측정각을 그대로 넣어
      // 오차를 0 으로 두었는데, 그러면 기동 목표(0°)가 적용되지 않았다.
      angle_ref = target_angle_slewed_[(size_t)a];
    }

    // 최초 진입: 속도 추정기 초기화만 하고 제어 출력은 건너뜀
    if (!state.initialized) {
      state.prev_angle    = angle;
      state.vel_filt      = 0.0;
      state.integral      = 0.0;
      state.p_pos_ref_filt = cfg.m1.p_bias_pos_kpa;
      state.p_neg_ref_filt = cfg.m1.p_bias_neg_kpa;
      state.initialized   = true;
      dbg_all.insert(dbg_all.end(), {angle, angle_ref, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
      continue;
    }

    // ── 각속도 추정 (유한차분 + LPF) ──
    const double vel_raw = (angle - state.prev_angle) / dt_sec;   // [deg/s]
    state.vel_filt = cfg.vel_filter_alpha * vel_raw
                    + (1.0 - cfg.vel_filter_alpha) * state.vel_filt;
    state.prev_angle = angle;
    const double vel = state.vel_filt;

    // ── PID ──
    // actuator_connected_=false 이면 엔코더가 실제로 움직이지 않아 error가 절대 해소되지
    // 않는다. 이 상태에서 PID를 그대로 돌리면 kp*error(비례항은 클램프 없음)가 각도 명령에
    // 비례해 무한정 커져 45°만 넘어도 p_pos_max_kpa에 곧장 포화되고, 45/60/90°가 전부 같은
    // 압력으로 뭉개진다 (20260818, 액추에이터 미연결 압력추종 테스트 중 확인).
    // → 액추에이터 미연결 시에는 PID를 끄고, 아래 중력 FF만으로 목표압력을 생성한다.
    // 모드 2 와 같은 처리 — 목표는 건드리지 않고 **제어 오차만** 밴드로 자른다.
    const double error_raw = angle_ref - angle;
    const double error = (target_follow_band_deg_ > 0.0)
        ? std::clamp(error_raw, -target_follow_band_deg_, target_follow_band_deg_)
        : error_raw;
    // D 항은 목표 속도를 빼고 본다 (명령한 움직임은 억제하지 않는다).
    const double vel_ref_m1 = (a < (int)target_slew_rate_.size() && pos_tcp_received_)
        ? target_slew_rate_[(size_t)a] : 0.0;
    const double vel_err_m1 = vel - kd_vel_ff_ * vel_ref_m1;
    double p_pid = 0.0;

    if (actuator_connected_) {
      // 적분 (부호 반전 시 리셋: 목표 반대 방향으로 쌓인 적분이 오버슈트를 유발하지 않도록)
      if ((error > 0.0 && state.integral < 0.0) ||
          (error < 0.0 && state.integral > 0.0)) {
        state.integral = 0.0;
      }
      // 적분 (와인드업 방지: 포화 전 클램핑)
      state.integral += error * dt_sec;
      const double integ_limit = (std::abs(cfg.m1.ki) > 1e-9)
                                 ? cfg.m1.integral_limit_kpa / cfg.m1.ki
                                 : 0.0;
      state.integral = std::clamp(state.integral, -integ_limit, integ_limit);

      p_pid = cfg.m1.kp * error
            + cfg.m1.ki * state.integral
            - cfg.m1.kd * vel_err_m1;   // 미분: 측정 − 목표 속도
    }

    // ── 중력 피드포워드 ──
    // τ = m·g·L·cos(90°-angle) = m·g·L·sin(angle_rad)
    // actuator_connected_=false: 실제 각도가 고정돼 있으므로 목표각(angle_ref) 기준으로
    // 계산해야 각도 명령이 실제로 서로 다른 목표압력에 매핑된다.
    const double ff_angle = actuator_connected_ ? angle : angle_ref;
    const double angle_rad = ff_angle * M_PI / 180.0;
    const double tau_gravity = cfg.mass_kg * 9.81
                              * cfg.link_length_m
                              * std::sin(angle_rad);          // [N·m]
    const double p_ff = cfg.m1.kff_gravity * tau_gravity;   // [kPa]

    // ── 마찰 보상 (쿨롱) ── error 방향으로 보상 (vel 방향은 수축 필요 시 역방향 힘을 줌)
    // 액추에이터 미연결 시에는 실제 움직임이 없으므로 마찰 보상도 의미가 없어 생략.
    double p_friction = 0.0;
    if (actuator_connected_ && std::abs(error) > 0.3) {
      p_friction = cfg.m1.friction_kpa * (error > 0.0 ? 1.0 : -1.0);
    }

    // ── 합산 및 압력 레퍼런스 생성 ──
    const double delta = p_pid + p_ff + p_friction;

    const double p_pos_unsat = cfg.m1.p_bias_pos_kpa + delta;
    double p_pos = std::clamp(p_pos_unsat, cfg.m1.p_pos_min_kpa, cfg.p_pos_max_kpa);

    double p_neg = std::clamp(
      cfg.m1.p_bias_neg_kpa - delta * cfg.m1.neg_coupling,
      cfg.p_neg_min_kpa, cfg.m1.p_neg_max_kpa);

    // P+ 포화 & 연장 방향 오차 시 음압 독립 구동
    // neg_coupling은 delta에 비례하므로 P+가 천장(anti-windup으로 delta 동결)에
    // 걸리면 P-도 같이 멈춤. error>0인 동안 P-를 p_neg_min으로 독립 구동해
    // 차압을 최대화한다.
    if (p_pos_unsat > cfg.p_pos_max_kpa && error > 0.0) {
      p_neg = cfg.p_neg_min_kpa;
    }

    // 포화 시 적분 되돌리기 (back-calculation anti-windup)
    // 오차 방향과 같은 방향으로 포화된 경우에만 취소:
    //   - 연장 필요(error>0)인데 p_pos가 최대에 걸림 → 더 밀어봤자 의미없음
    //   - 수축 필요(error<0)인데 p_pos가 최소에 걸림 → 중력이 이미 수축 중, 적분은 계속 쌓음
    const bool sat_same_dir = (p_pos_unsat > cfg.p_pos_max_kpa && error > 0.0) ||
                              (p_pos_unsat < cfg.m1.p_pos_min_kpa && error < 0.0);
    if (actuator_connected_ && sat_same_dir && std::abs(cfg.m1.ki) > 1e-9) {
      state.integral -= error * dt_sec;   // 이번 적분 취소
    }

    // ── 압력 레퍼런스 슬루레이트 제한 ──
    // p_pos/p_neg가 한 tick에서 크게 점프하면(예: 위치명령 변경) 밸브모델-실제 불일치로
    // 큰 실압력 스파이크가 발생할 수 있으므로, MPC에 넘기는 레퍼런스 자체를
    // ref_slew_kpa_per_s 로 제한된 램프로 바꿔 서서히 목표에 도달하게 한다.
    const double max_step = cfg.m1.ref_slew_kpa_per_s * dt_sec;
    state.p_pos_ref_filt += std::clamp(p_pos - state.p_pos_ref_filt, -max_step, max_step);
    state.p_neg_ref_filt += std::clamp(p_neg - state.p_neg_ref_filt, -max_step, max_step);
    p_pos = state.p_pos_ref_filt;
    p_neg = state.p_neg_ref_filt;

    // mpc_ref_kpa_ 에 기록 (MPC 내층이 이 값을 압력 레퍼런스로 사용)
    {
      std::lock_guard<std::mutex> lk(mpc_ref_mtx_);
      const int pg = cfg.pos_gid;
      const int ng = cfg.neg_gid;
      if (pg >= 0 && pg < (int)mpc_ref_kpa_.size()) mpc_ref_kpa_[pg] = p_pos;
      if (ng >= 0 && ng < (int)mpc_ref_kpa_.size()) mpc_ref_kpa_[ng] = p_neg;
    }

    dbg_all.insert(dbg_all.end(), {angle, angle_ref, p_pos, p_neg, p_pid, p_ff, p_friction, vel});

    // 500Hz × 250 = 0.5초마다 출력
    if (tick_ % 250 == 0) {
      RCLCPP_INFO(get_logger(),
        "[PosCtrl axis%d] θ=%.2f°  ref=%.2f°  err=%+.2f°  vel=%+.1fdps | "
        "pid=%+.1f ff=%.1f fric=%+.1f → P+=%.1f  P-=%.1f kPa",
        a, angle, angle_ref, error, vel, p_pid, p_ff, p_friction, p_pos, p_neg);
    }
  }

  // 디버그 토픽 발행 (500Hz → 구독 측에서 다운샘플 권장)
  // 축마다 8개씩 이어붙임: [angle, angle_ref, p_pos_ref, p_neg_ref, p_pid, p_ff, p_friction, vel_dps] × n
  if (pub_pos_dbg_) {
    std_msgs::msg::Float64MultiArray dbg;
    dbg.data = dbg_all;
    pub_pos_dbg_->publish(dbg);
  }
}

// ================================
// run_optimized_pressure_ref  (control_mode 2)
// ================================
// 위치 PID → 목표 토크 → PressureRefGen → 12개 목표 압력 + 적응 레일 셋포인트
//
// mode 1 과의 차이:
//   - PID 출력이 kPa 가 아니라 **토크(N·m)** 다. 중력 FF 가 이미 m·g·L·sinθ 로 토크를
//     계산하고 있었으므로 kff_gravity 곱셈만 빼면 그대로 정확한 FF 가 된다
//     (기존 kff=3.0 은 물리값 20.4 의 1/6.8 이라 90°에서 중력의 29% 만 보상했다).
//   - 양압/음압 분배가 고정 bias±delta 가 아니라, 그 순간 밸브에 실제 흐르는 유량으로
//     만든 슬루 박스 안에서 최적화로 결정된다.
//   - 라인압 셋포인트가 고정 상수가 아니라 수요에 따라 능력경계 위에서 재배분된다.
//
// 생성기는 gen_period_ms_ (기본 20 ms) 마다 한 번만 돌리고, 그 사이에는 결과를 유지한다
// (레일은 초 단위로 느리고 챔버는 20 ms 안에 수십 kPa 움직이므로 계층 분리가 성립).
void Controller::run_optimized_pressure_ref(double dt_sec)
{
  if (!refgen_) return;
  const int N = num_actuators_;
  const double atm = sensor_.kpa_atm();
  auto to_gauge_pa = [atm](double abs_kpa) { return (abs_kpa - atm) * 1000.0; };
  auto to_abs_kpa  = [atm](double gauge_pa) { return gauge_pa / 1000.0 + atm; };

  // ── 1. 축별 목표 토크 (위치 PID + 중력 FF + 마찰) ───────────────────
  std::array<double, 9> ang;
  {
    std::lock_guard<std::mutex> lk(sensors_mtx_);
    ang = encoder_angles_;
  }
  std::vector<double> F_ref((size_t)N, 0.0), tau_ref((size_t)N, 0.0);
  std::vector<double> dbg_tau_pid((size_t)N, 0.0), dbg_tau_ff((size_t)N, 0.0);
  std::vector<double> dbg_angle((size_t)N, 0.0), dbg_angle_ref((size_t)N, 0.0);
  std::vector<double> dbg_vel((size_t)N, 0.0);

  const double reel_m = reel_radius_mm_ * 1e-3;

  for (int a = 0; a < N; ++a) {
    auto& cfg   = pos_ctrl_cfg_[(size_t)a];
    auto& state = pos_ctrl_state_[(size_t)a];
    auto& tp    = tau_pid_[(size_t)a];

    const int enc = std::clamp(cfg.actuator_idx, 0, (int)ang.size() - 1);
    const double angle = ang[(size_t)enc];
    double angle_ref;
    {
      std::lock_guard<std::mutex> lk(mpc_ref_mtx_);
      // 항상 슬루된 목표를 쓴다. 예전에는 TCP 수신 전에 측정각을 그대로 넣어
      // 오차를 0 으로 두었는데, 그러면 기동 목표(0°)가 적용되지 않았다.
      angle_ref = target_angle_slewed_[(size_t)a];
    }

    if (!state.initialized) {
      state.prev_angle = angle; state.vel_filt = 0.0; state.initialized = true;
      tau_integ_[(size_t)a] = 0.0;
    }
    const double vel_raw = (angle - state.prev_angle) / dt_sec;
    state.vel_filt = cfg.vel_filter_alpha * vel_raw + (1.0 - cfg.vel_filter_alpha) * state.vel_filt;
    state.prev_angle = angle;
    const double vel = state.vel_filt;

    const double err_raw = angle_ref - angle;

    // ── 추종 오차 제한 ────────────────────────────────────────────────
    // **목표 자체는 건드리지 않는다.** 예전에는 target_angle_slewed_ 를 측정각
    // ±밴드로 묶었는데, 그러면 팔이 멈춘 자리에서 목표까지 같이 얼어붙는다 —
    // 80° 에서 0° 를 명령하면 팔이 14° 에 서고 목표는 9° 에서 멈췄다
    // (실기 20260829_193558: angle − target 이 정확히 +5.00 에 붙어 있었다).
    // 목표는 명령까지 끝까지 가야 한다. 묶어야 하는 것은 **제어기에 들어가는
    // 오차** 뿐이다 — 적분 와인드업과 도달 못 할 큰 수요를 막는 것이 목적이니
    // 여기서 자르면 충분하다.
    const double err = (target_follow_band_deg_ > 0.0)
        ? std::clamp(err_raw, -target_follow_band_deg_, target_follow_band_deg_)
        : err_raw;

    // ── 오차가 밴드에 계속 붙어 있으면 알린다 ────────────────────────────
    //
    // 밴드에 붙으면 kp·err 이 상수가 되고, 적분도 integ_limit_nm 에서 멈추고,
    // 마찰항도 포화한다 — **목표 각도가 더 이상 압력 레퍼런스에 실리지 않는다.**
    // 45° 든 90° 든 같은 토크를 요구하게 된다.
    //
    // 액추에이터를 떼고 시험할 때 특히 그렇다: 팔이 안 움직이니 오차가 절대
    // 안 줄고, 0.75 s (= (integ_limit/ki)/band) 만에 전부 포화한다. 실기
    // 20260829_220606 (actuator_connected:=true, 액추에이터 미연결):
    //     45° → τ 1.90 N·m,  90° → τ 1.92 N·m   (구분이 사라졌다)
    // 그때는 **actuator_connected:=false** 로 돌려야 한다 — PID 를 끄고 중력 FF 를
    // 목표각으로 계산하므로 각도 명령이 그대로 압력에 실린다.
    if (actuator_connected_ && target_follow_band_deg_ > 0.0 &&
        std::abs(err_raw) > target_follow_band_deg_ * 1.5) {
      if (++band_sat_ticks_[(size_t)a] > (int)(3.0 / std::max(1e-6, dt_sec))) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "[axis%d] 추종 오차가 3 초 넘게 밴드(%.1f°)에 붙어 있다 — 실제 %.1f°. "
          "이 상태에서는 kp·오차·적분·마찰이 모두 포화해 **목표 각도가 압력 "
          "레퍼런스에 반영되지 않는다** (45° 와 90° 가 같은 토크가 된다). "
          "액추에이터가 안 붙어 있다면 actuator_connected:=false 로 돌릴 것 — "
          "그러면 중력 FF 를 목표각으로 계산해 각도가 그대로 압력에 실린다.",
          a, target_follow_band_deg_, err_raw);
        band_sat_ticks_[(size_t)a] = 0;
      }
    } else {
      band_sat_ticks_[(size_t)a] = 0;
    }

    // 적분 (부호 반전 시 리셋 + 클램프)
    double& I = tau_integ_[(size_t)a];
    if ((err > 0.0 && I < 0.0) || (err < 0.0 && I > 0.0)) I = 0.0;
    if (actuator_connected_) I += err * dt_sec;
    const double I_lim = (std::abs(tp.ki) > 1e-12) ? tp.integ_limit_nm / tp.ki : 0.0;
    I = std::clamp(I, -I_lim, I_lim);

    // ── D 항: 목표 속도 피드포워드 ────────────────────────────────────
    //
    // 예전에는 −kd·vel 이라 **명령한 움직임까지 억제**했다. 목표가 −15 deg/s 로
    // 내려가라고 하는데 팔이 그대로 내려가면 D 가 +0.3 N·m 로 붙잡는다. 그래서
    // 팔이 못 내려가다가, 중력이 이겨 −45 deg/s 로 미끄러지면 D 가 뒤늦게
    // +0.9 로 튀어 붙잡는다 — 계단의 정체다.
    //
    // 실기 20260829_193558 하강(t 8~18): 목표는 15 deg/s 인데 실제 속도가
    // −63.8 ~ +14.1 deg/s 로 요동했고 **15% 는 하강 중에 위로 되튀었다**.
    // 그 순간 τ 는 1.2 → 3.5 N·m 로 튀었는데 챔버는 128 kPa 에서 못 따라와
    // (목표 156) 붙잡기가 늦고 약했다.
    //
    // vel 대신 (vel − 목표속도) 를 쓰면 D 는 **명령에서 벗어난 만큼만** 억제한다.
    // 명령한 속도로 내려가는 동안은 0 이고, −45 로 미끄러지면 그 초과분만 잡는다.
    // kd_vel_ff_ 0 이면 예전 동작, 1 이면 완전 피드포워드.
    const double vel_ref = (a < (int)target_slew_rate_.size() && pos_tcp_received_)
        ? target_slew_rate_[(size_t)a] : 0.0;
    const double vel_err = vel - kd_vel_ff_ * vel_ref;

    const double tau_pid = actuator_connected_
        ? (tp.kp * err + tp.ki * I - tp.kd * vel_err) : 0.0;

    // 중력 피드포워드. tau_ff_gain 으로 크기를 줄일 수 있다.
    //
    // 액추에이터를 떼고 압력만 시험할 때는 실제로 들 하중이 없다. 그런데 기하값
    // (5 kg × 0.15 m)이 그대로 들어가면 45° 에서 5.20 N·m → 208 N 을 요구하고,
    // 그 힘은 양압 정격(185 kPa abs) 단독으로 못 내므로 생성기가 음압까지 30 kPa
    // 까지 끌어내린다. 시험용으로는 과하다.
    // 게인은 **목표 압력에 거의 선형**으로 반영된다 (F = P⁺·A − P⁻·A).
    // 액추에이터를 붙이면 반드시 1.0 으로 되돌릴 것 — 그때는 중력을 실제로 들어야 한다.
    const double ff_angle = actuator_connected_ ? angle : angle_ref;
    const double tau_grav = tp.tau_ff_gain * cfg.mass_kg * 9.81 * cfg.link_length_m
                          * std::sin(ff_angle * M_PI / 180.0);

    // 마찰 보상. **하드 sign 은 err 이 0 을 지날 때마다 ±friction_nm 를 통째로
    // 뒤집는다** — friction_nm 0.48 이면 0.96 N·m 계단이고, 이는 2 kg·150 mm 의
    // 중력 최대치(2.94 N·m)의 33% 다. 목표 근처에서 이게 매 틱 진동한다.
    // 밴드 안에서 선형으로 준다 (밴드 밖에서는 예전과 같은 ±friction_nm).
    const double fb = std::max(1e-6, tp.friction_band_deg);
    const double tau_fric = actuator_connected_
        ? tp.friction_nm * std::clamp(err / fb, -1.0, 1.0) : 0.0;

    // 이 시스템은 한 방향 힘만 낸다 → 목표는 항상 ≥ 0
    tau_ref[(size_t)a] = std::max(0.0, tau_pid + tau_grav + tau_fric);
    F_ref[(size_t)a]   = tau_ref[(size_t)a] / std::max(1e-6, reel_m);

    dbg_tau_pid[(size_t)a] = tau_pid;
    dbg_tau_ff[(size_t)a]  = tau_grav;
    dbg_angle[(size_t)a]   = angle;
    dbg_angle_ref[(size_t)a] = angle_ref;
    dbg_vel[(size_t)a]     = vel;
  }

  // ── 2. 생성기 주기마다 최적화 1회 ───────────────────────────────────
  const int decim = std::max(1, gen_period_ms_ / std::max(1, period_ms_));
  const bool run_gen = (gen_tick_ % (uint64_t)decim == 0) || !gen_has_result_;
  ++gen_tick_;

  PressureRefGen::Result r;
  if (run_gen) {
    // 공급원: boards 1~4 측정값
    PressureRefGen::SupplyState sup;
    sup.P_rail_pos = to_gauge_pa(filt_out_[P_pos_board_id_ - 1]);
    sup.P_rail_neg = to_gauge_pa(filt_out_[P_neg_board_id_ - 1]);
    sup.P_tank     = to_gauge_pa(filt_out_[P_macro_board_id_ - 1]);
    sup.P_ej       = to_gauge_pa(filt_out_[P_macro_neg_board_id_ - 1]);
    sup.use_ej_meas = gen_use_ej_meas_;
    // MacroSwitch(board4 v1) 개방 여부 = 이젝터 구동 중
    sup.ej_running  = (macro_switch_pwm_index_ >= 0 && macro_switch_pwm_index_ < PWM_TOTAL)
                      ? (zoh_[(size_t)macro_switch_pwm_index_] > 0) : false;

    // 축 상태: 챔버 압력 + 부피 + 부피 변화율
    std::vector<PressureRefGen::AxisState> axes((size_t)N);
    for (int a = 0; a < N; ++a) {
      const auto& cfg = pos_ctrl_cfg_[(size_t)a];
      const int pos_bid = cfg.pos_gid + channel_board_offset_;
      const int neg_bid = cfg.neg_gid + channel_board_offset_;
      auto& ax = axes[(size_t)a];
      ax.P_pos = to_gauge_pa(filt_out_[std::clamp(pos_bid - 1, 0, NUM_CAN_BOARDS - 1)]);
      ax.P_neg = to_gauge_pa(filt_out_[std::clamp(neg_bid - 1, 0, NUM_CAN_BOARDS - 1)]);
      ax.V_pos = std::max(1e-9, vol_ml_[(size_t)cfg.pos_gid] * 1e-6);
      ax.V_neg = std::max(1e-9, vol_ml_[(size_t)cfg.neg_gid] * 1e-6);
      // V̇ = A · reel · ω  (신장 방향이 양압 챔버를 키운다)
      const double omega_rad = dbg_vel[(size_t)a] * M_PI / 180.0;
      const double dV = piston_area_mm2_ * reel_radius_mm_ * omega_rad * 1e-9;  // mm³/s → m³/s
      ax.dVdt_pos =  dV;
      ax.dVdt_neg = -dV;
    }

    r = refgen_->step(F_ref, axes, sup);

    // ── 압력 레퍼런스 슬루 제한 ────────────────────────────────────────
    //
    // 모드 1 에는 ref_slew_kpa_per_s 가 있었는데 모드 2 에는 없었다. 생성기 출력이
    // 그대로 MPC 로 갔고, 힘 수요가 조금만 떨리면 레퍼런스가 통째로 튀었다.
    // 실기 20260829_165306: 40 ms 사이에 P⁻ 레퍼런스가 101.3 → 51.3 → 101.0 kPa
    // (≈1250 kPa/s), P⁺ 가 101.5 → 122.1 → 105.4 였다. 챔버가 따라갈 수 없는
    // 명령이라 밸브만 두들기고 액추에이터에는 충격으로 간다.
    //
    // 0 이하면 끔(예전 동작).
    const double gen_dt = std::max(1e-3, gen_period_ms_ / 1000.0);
    const double max_step = gen_ref_slew_kpa_s_ * gen_dt;
    for (int a = 0; a < N; ++a) {
      const double want_p = to_abs_kpa(r.P_pos_ref[(size_t)a]);
      const double want_n = to_abs_kpa(r.P_neg_ref[(size_t)a]);
      if (gen_ref_slew_kpa_s_ > 0.0 && gen_has_result_) {
        gen_pos_ref_kpa_[(size_t)a] +=
            std::clamp(want_p - gen_pos_ref_kpa_[(size_t)a], -max_step, max_step);
        gen_neg_ref_kpa_[(size_t)a] +=
            std::clamp(want_n - gen_neg_ref_kpa_[(size_t)a], -max_step, max_step);
      } else {
        gen_pos_ref_kpa_[(size_t)a] = want_p;
        gen_neg_ref_kpa_[(size_t)a] = want_n;
      }
    }
    // ── 공급이 없어 레퍼런스를 못 만드는 상태를 **직접** 알린다 ──────────
    //
    // 힘은 요구되는데 챔버 상한(ub⁺)이 대기압에 붙어 있으면 레퍼런스가 대기압에서
    // 못 움직인다. 로그에는 "P⁺ 레퍼런스가 101.3 고정" 으로만 보여서 제어 버그처럼
    // 읽힌다 — 실제로는 **레일에 공기가 없다는** 뜻이다.
    // 20260829_222722 에서 탱크가 t=4.84 s 에 585 → 97 kPa 로 빠졌고(그 순간 부스트
    // 0.000 g/s, 이젝터 0.000 g/s — 컨트롤러는 탱크를 안 쓰고 있었다) 그 뒤 모든
    // 실험에서 레퍼런스가 대기압에 붙었다.
    {
      const double atm_g = 0.0;   // gauge 기준 대기압
      double want = 0.0, head = 1e18;
      for (int a = 0; a < N; ++a) {
        want = std::max(want, tau_ref[(size_t)a]);
        head = std::min(head, r.ub_pos[(size_t)a] - atm_g);
      }
      if (want > 0.2 && head < 2000.0) {   // 힘은 요구되는데 상한이 대기압 +2 kPa 미만
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
          "레퍼런스를 못 만든다: 토크 %.2f N·m 가 요구되는데 챔버 상한이 %.1f kPa abs "
          "(대기압) 다. 양압레일 %.1f / 탱크 %.1f kPa abs — **공급에 공기가 없다.** "
          "레퍼런스 생성기는 레일이 못 주는 압력을 목표로 삼지 않는다(정상 동작). "
          "컴프레서·탱크 배관을 확인할 것.",
          want, to_abs_kpa(head + atm_g),
          filt_out_[P_pos_board_id_ - 1], filt_out_[P_macro_board_id_ - 1]);
      }
    }

    gen_rail_pos_sp_kpa_ = to_abs_kpa(r.rail_pos_sp);
    gen_rail_neg_sp_kpa_ = to_abs_kpa(r.rail_neg_sp);
    gen_has_result_ = true;

    // 부족률은 **진단값**이다 (MATLAB 도 usage 를 버린다: `[rail_next, ~] = ...`).
    // macro 개방은 여기서 정하지 않는다 — 내층이 매 틱 유량을 나눠 저절로 결정한다.
    for (int a = 0; a < N; ++a) {
      gen_starve_pos_[(size_t)a] = r.starve_pos[(size_t)a] * 100.0;
      gen_starve_neg_[(size_t)a] = r.starve_neg[(size_t)a] * 100.0;
    }

    // ── 3. 적응 레일 셋포인트를 LinePID 에 넘긴다 ────────────────────
    pid_pos_.ref = gen_rail_pos_sp_kpa_;
    pid_neg_.ref = gen_rail_neg_sp_kpa_;

    // 디버그 토픽: 축마다 10개 + 말미 공용 6개
    if (pub_refgen_dbg_) {
      std_msgs::msg::Float64MultiArray m;
      m.data.reserve((size_t)N * 12 + 6);
      for (int a = 0; a < N; ++a) {
        m.data.push_back(dbg_angle[(size_t)a]);
        m.data.push_back(dbg_angle_ref[(size_t)a]);
        m.data.push_back(tau_ref[(size_t)a]);
        m.data.push_back(r.F_achieved[(size_t)a] * reel_m);      // 달성 토크 [N·m]
        m.data.push_back(gen_pos_ref_kpa_[(size_t)a]);
        m.data.push_back(gen_neg_ref_kpa_[(size_t)a]);
        m.data.push_back(to_abs_kpa(r.ub_pos[(size_t)a]));       // 슬루 상한 P⁺
        m.data.push_back(to_abs_kpa(r.lb_pos[(size_t)a]));
        m.data.push_back(to_abs_kpa(r.lb_neg[(size_t)a]));       // 슬루 하한 P⁻
        m.data.push_back(to_abs_kpa(r.ub_neg[(size_t)a]));
        m.data.push_back(gen_starve_pos_[(size_t)a]);             // 양압 유량 부족률 [%]
        m.data.push_back(gen_starve_neg_[(size_t)a]);             // 음압 유량 부족률 [%]
      }
      m.data.push_back(gen_rail_pos_sp_kpa_);
      m.data.push_back(gen_rail_neg_sp_kpa_);
      m.data.push_back(filt_out_[P_macro_board_id_ - 1]);        // 탱크 압력 [kPa abs]
      m.data.push_back(r.tank_low ? 1.0 : 0.0);
      m.data.push_back(r.m_boost * 1e3);                        // 부스트 [g/s]
      m.data.push_back(r.m_eject * 1e3);                        // 이젝터 [g/s]
      pub_refgen_dbg_->publish(m);
    }

    // ── 공급 부족 경고 ────────────────────────────────────────────────
    //
    // 공급이 모자라면 밸브를 아무리 열어도 챔버가 안 찬다. 그 상태의 로그로
    // 밸브를 피팅하면 실제의 수십분의 1 로 잡혀 컨트롤러가 전 채널을 100% 로
    // 내게 된다 — 실기 20260828_181748 에서 탱크가 98 kPa(정상 ~600)까지
    // 비어 있었는데 " LOW" 가 INFO 줄 끝에 붙어 있을 뿐이라 알아채지 못했다.
    // 터미널에서 바로 보이게 ERROR 로 올린다 (2 초마다 한 번).
    {
      const double tank_abs = filt_out_[P_macro_board_id_ - 1];
      const double rail_abs = filt_out_[P_pos_board_id_ - 1];
      const double rail_neg_abs = filt_out_[P_neg_board_id_ - 1];
      double ref_pos_max = 0.0, ref_neg_min = 1e9;
      for (int a = 0; a < N; ++a) {
        ref_pos_max = std::max(ref_pos_max, gen_pos_ref_kpa_[(size_t)a]);
        ref_neg_min = std::min(ref_neg_min, gen_neg_ref_kpa_[(size_t)a]);
      }
      if (r.tank_low) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
          "[공급 부족] 탱크 %.0f kPa — 운전 하한 %.0f kPa 미만이다. macro 부스트를 쓸 수 없다. "
          "컴프레서/펌프를 확인할 것. 이 상태의 로그는 밸브 피팅에 쓰면 안 된다.",
          tank_abs, get_param_or<double>(this, "PressureRefGen.tank_stop_kpa", 450.0) + 101.325);
      }
      // 레일이 챔버 목표보다 낮으면 micro 로는 채울 방법이 없다.
      // 대기압 근처 목표(=사실상 수요 없음)에서는 알리지 않는다. 기동 직후
      // 챔버·레일이 모두 101 kPa 일 때 "여유 +1 kPa" 라고 뜨는 오탐이 있었다.
      if (ref_pos_max > 101.325 + 5.0 && rail_abs < ref_pos_max + 3.0) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
          "[공급 부족] 양압레일 %.0f kPa 로는 챔버 목표 최대 %.0f kPa 를 채울 수 없다 "
          "(여유 %+.0f kPa). 수요가 macro 로 몰린다. 레일 셋포인트는 %.0f kPa 다. "
          "이 라인 밸브는 배기(레일→대기)라 레일을 올릴 수 없다 — 펌프 능력을 확인할 것.",
          rail_abs, ref_pos_max, rail_abs - ref_pos_max, gen_rail_pos_sp_kpa_);
      }
      // 여유가 **양수면 정상**이다. 문턱을 3 kPa 로 두어 진짜 부족할 때만 알린다.
      // (10 kPa 로 두었더니 여유 8.9 kPa 인 정상 상태에도 "깊지 않다" 고 떠서
      //  오탐이었다.)
      if (ref_neg_min < 101.325 - 5.0 && rail_neg_abs > ref_neg_min - 3.0) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
          "[공급 부족] 음압레일 %.1f kPa 로는 챔버 목표 최저 %.1f kPa 에 못 미친다 "
          "(여유 %+.1f kPa). 레일 셋포인트는 %.1f kPa 다.",
          rail_neg_abs, ref_neg_min, ref_neg_min - rail_neg_abs, gen_rail_neg_sp_kpa_);
      }
    }

    if (tick_ % 250 == 0) {
      RCLCPP_INFO(get_logger(),
        "[RefGen] θ=%.2f→%.2f°  τ=%.2f/%.2f N·m (pid %.2f ff %.2f) | "
        "P⁺=%.1f[%.1f~%.1f] P⁻=%.1f[%.1f~%.1f] | rail SP %.1f/%.1f | "
        "tank %.0f%s starve %.0f/%.0f%% [%s%s] it=%d",
        dbg_angle[0], dbg_angle_ref[0], r.F_achieved[0] * reel_m, tau_ref[0],
        dbg_tau_pid[0], dbg_tau_ff[0],
        gen_pos_ref_kpa_[0], to_abs_kpa(r.lb_pos[0]), to_abs_kpa(r.ub_pos[0]),
        gen_neg_ref_kpa_[0], to_abs_kpa(r.lb_neg[0]), to_abs_kpa(r.ub_neg[0]),
        gen_rail_pos_sp_kpa_, gen_rail_neg_sp_kpa_,
        filt_out_[P_macro_board_id_ - 1], r.tank_low ? " LOW" : "",
        gen_starve_pos_[0], gen_starve_neg_[0],
        r.starve_pos[0] > 0.0 ? "B" : "-",
        r.starve_neg[0] > 0.0 ? "E" : "-",
        r.sqp_iters);
    }
  }

  // ── 4. 결과를 MPC 레퍼런스로 (생성기 주기 사이에는 ZOH) ─────────────
  {
    std::lock_guard<std::mutex> lk(mpc_ref_mtx_);
    for (int a = 0; a < N; ++a) {
      const auto& cfg = pos_ctrl_cfg_[(size_t)a];
      if (cfg.pos_gid >= 0 && cfg.pos_gid < (int)mpc_ref_kpa_.size())
        mpc_ref_kpa_[(size_t)cfg.pos_gid] = gen_pos_ref_kpa_[(size_t)a];
      if (cfg.neg_gid >= 0 && cfg.neg_gid < (int)mpc_ref_kpa_.size())
        mpc_ref_kpa_[(size_t)cfg.neg_gid] = gen_neg_ref_kpa_[(size_t)a];
    }
  }
}

AcadosMpc* Controller::mpc_for_gid(int gid) const {
  for (const auto& m : mpcs_)
    if (m && m->cfg().global_id == gid) return m.get();
  return nullptr;
}

void Controller::inner_loop_1khz(float /*dt_ms*/) {
  std::fill(inner_.begin(), inner_.end(), 0);
}

void Controller::publish_cmds() {
  std_msgs::msg::UInt16MultiArray m;
  m.data.assign(cmds_.begin(), cmds_.end());
  pub_pwm_cmd_->publish(m);
}