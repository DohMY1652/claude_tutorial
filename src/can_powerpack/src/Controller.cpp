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
  const double I_MAX_d   = (double)cfg_.I_MAX;
  const double A_max_d   = (double)cfg_.A_max;
  const double k_shape_d = (double)cfg_.k_shape;
  const double C_k_d     = (double)cfg_.C_k;
  const double C_p_d     = (double)cfg_.C_p;
  const double C_z_d     = (double)cfg_.C_z;
  const double alpha_d   = (double)cfg_.alpha_shape;

  // Compressible flow Phi helper (kappa=1.4)
  auto get_phi_local = [](double Pin, double Pout) -> double {
    if (Pin < 1e-9 || Pout >= Pin) return 0.0;
    constexpr double kappa = 1.4;
    const double Pr  = std::clamp(Pout / Pin, 0.0, 1.0);
    const double Pcr = std::pow(2.0 / (kappa + 1.0), kappa / (kappa - 1.0));
    if (Pr <= Pcr)
      return std::sqrt(kappa * std::pow(2.0/(kappa+1.0), (kappa+1.0)/(kappa-1.0)));
    return std::sqrt(2.0*kappa/(kappa-1.0)) * std::sqrt(std::max(0.0,
      std::pow(Pr, 2.0/kappa) - std::pow(Pr, (kappa+1.0)/kappa)));
  };

  // Static flow Q_static [LPM] for the 13-var model (dynamics frozen at current z)
  auto Q_static_fn = [&](double u_pct, double Pin, double Pout, double z_val) -> double {
    u_pct = std::clamp(u_pct, 0.0, 100.0);
    const double I         = u_pct / 100.0 * I_MAX_d;
    const double Force_net = std::clamp(I + C_z_d*z_val + C_p_d*Pin - C_k_d, -500.0, 500.0);
    const double sigma     = 1.0 / (1.0 + std::exp(-k_shape_d * Force_net));
    const double Area_eff  = A_max_d * std::pow(sigma, alpha_d);
    return Area_eff * Pin * get_phi_local(Pin, Pout);
  };

  // Numerical Jacobian of Q_static → [dQ/du_pct, dQ/dPin, dQ/dPout] * scale
  // Matches the original calc_rounds return convention: [round_input, round_pin, round_pout]
  auto calc_rounds = [&](double input, double Pin, double Pout, double z_val)
  {
    input = std::clamp(input, 0.0, 100.0);
    const double Q0 = Q_static_fn(input, Pin, Pout, z_val);

    constexpr double du = 0.5, dP = 0.5;
    const double dQ_du   = (Q_static_fn(std::min(input+du, 100.0), Pin,    Pout,    z_val) - Q0) / du;
    const double dQ_dPin = (Q_static_fn(input, Pin + dP, Pout,    z_val) - Q0) / dP;
    const double dQ_dPout= (Q_static_fn(input, Pin,    Pout + dP, z_val) - Q0) / dP;

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
    return calc_rounds(input, P_chamber, P_limit, z_macro_);
  };

  const double u_mi = std::clamp((double)u_ref(0), 0.0, 100.0);
  const double u_ma = std::clamp((double)u_ref(1), 0.0, 100.0);
  const double u_at = std::clamp((double)u_ref(2), 0.0, 100.0);
  const double leak_u_pos = (double)cfg_.leakage_u_pos;
  const double leak_u_neg = (double)cfg_.leakage_u_neg;

  double A_scalar = 0.0;
  Eigen::RowVector3f B_row; B_row.setZero();

  if (cfg_.is_positive) {
    auto mi = calc_rounds(u_mi, P_micro, P_now,  z_micro_);
    auto ma = calc_rounds(u_ma, P_macro, P_now,  z_macro_);
    auto at = calc_rounds(u_at, P_now,   P_atm,  z_atm_);
    auto lk = calc_rounds(leak_u_pos, P_now, P_atm, z_atm_);

    const double tmp_A = mi[2] + ma[2] - at[1] - lk[1];
    const double b0 =  mi[0];
    const double b1 =  ma[0];
    const double b2 = -at[0];

    A_scalar = (float)tmp_A;
    B_row << (float)b0, (float)b1, (float)b2;

  } else {
    auto at = calc_rounds(u_at, P_atm,  P_now,  z_atm_);
    auto mi = calc_rounds(u_mi, P_now,  P_micro, z_micro_);
    auto ma = ejector_calc_rounds(u_ma, P_now);
    auto lk = calc_rounds(leak_u_neg, P_atm, P_now, z_atm_);

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
  auto update_bw = [&](double& z, double& prev_I, int& dir, double u_pct) {
    const double I     = u_pct / 100.0 * (double)cfg_.I_MAX;
    const double dI    = I - prev_I;
    const double abs_dI = std::abs(dI);
    z += (double)cfg_.A_bw * dI
       - (double)cfg_.beta_bw  * abs_dI * z
       - (double)cfg_.gamma_bw * dI * std::abs(z);
    z = std::clamp(z, -1e6, 1e6);
    if      (dI >  1e-4) dir = 1;
    else if (dI < -1e-4) dir = 0;
    prev_I = I;
  };
  update_bw(z_micro_, prev_I_micro_, dir_micro_, (double)last_u3_[0]);
  update_bw(z_atm_,   prev_I_atm_,   dir_atm_,   (double)last_u3_[2]);
  update_bw(z_macro_, prev_I_macro_, dir_macro_, (double)last_u3_[1]);

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

  // 유효면적 → 필요 전류[%] 역산. 순방향 모델
  //   F_net = I + C_z·z + C_p·Pin − C_k ,  A_eff = A_max·sigmoid(k·F_net)^alpha
  // 을 그대로 뒤집은 것이다. C_p·Pin 항 때문에 **상류 압력이 높으면 필요 전류가 낮아진다**
  // (압력이 스풀을 밀어 올려 자기력을 돕는다).
  auto u_of_area = [&](double Area_req, double Pin, double z_val) -> float {
    const double A_max_d = (double)cfg_.A_max;
    const double alpha_d = (double)cfg_.alpha_shape;
    if (Area_req >= A_max_d) return 100.0f;
    const double sigma = std::pow(std::clamp(Area_req / A_max_d, 1e-9, 1.0-1e-9), 1.0/alpha_d);
    const double F_req = std::log(sigma / (1.0 - sigma)) / (double)cfg_.k_shape;
    const double I_req = F_req - (double)cfg_.C_z * z_val - (double)cfg_.C_p * Pin + (double)cfg_.C_k;
    return (float)std::clamp(I_req / (double)cfg_.I_MAX * 100.0, 0.0, 100.0);
  };

  // Inverse valve model: given required Q [LPM], P_in, P_out, z → u_pct [0,100]
  auto valve_invert = [&](double Q_req, double Pin, double Pout, double z_val) -> float {
    if (Q_req <= 0.0) return 0.0f;
    const double phi = get_phi_ff(Pin, Pout);
    if (phi < 1e-9) return 0.0f;
    return u_of_area(Q_req / (Pin * phi), Pin, z_val);
  };

  // 크래킹 임계 [%] — 이 명령 이하에서는 스풀이 들리지 않아 유량이 0 이다.
  // 실측(50% 부근)과 일치한다: 351 kPa abs 레일에서 이 모델은 약 52% 를 준다.
  auto u_crack = [&](double Pin, double z_val) {
    return u_of_area((double)cfg_.valve_crack_area_frac * (double)cfg_.A_max, Pin, z_val);
  };

  const float Pref = cfg_.ref_value;
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

  float target_time_constant = cfg_.target_time_constant;
  if (target_time_constant <= 0.001f) target_time_constant = 0.2f;
  
  const float P_abs_atm = current_P_atm_; 
  
  // P_dot 계산 (Feedforward)
  float P_dot = (Pref - P_now) * 1000 / target_time_constant; //[Pa/sec]

  float m_dot_pressure =  P_dot  * current_vol / (Rgas * TempK) / lpm2kgps; //[LPM]
  float m_dot_volume = P_now * 1000 * vol_dot / (Rgas * TempK) / lpm2kgps; //[LPM]
  
  // 적분기(Integral) 업데이트
  constexpr float integral_limit = 1000.0f;
  if (cfg_.is_positive) {
    if ((err > 0.0f && pos_error_integral_ < 0.0f) || (err < 0.0f && pos_error_integral_ > 0.0f)) {
      pos_error_integral_ = 0.0f;
    }
    pos_error_integral_ += err * dt_sec;
    pos_error_integral_ = std::clamp(pos_error_integral_, -integral_limit, integral_limit);
  } else {
    if ((err > 0.0f && neg_error_integral_ > 0.0f) || (err < 0.0f && neg_error_integral_ < 0.0f)) {
      neg_error_integral_ = 0.0f;
    }
    neg_error_integral_ -= err * dt_sec;
    neg_error_integral_ = std::clamp(neg_error_integral_, -integral_limit, integral_limit);
  }
  
  const float ki_mi = cfg_.is_positive ? cfg_.pos_ki_micro : cfg_.neg_ki_micro;
  const float ki_ma = cfg_.is_positive ? cfg_.pos_ki_macro : cfg_.neg_ki_macro;
  const float ki_at = cfg_.is_positive ? cfg_.pos_ki_atm   : cfg_.neg_ki_atm;
  
  float u_mi_req = 0.f, u_ma_req = 0.f, u_at_req = 0.f;

  // macro 개방 판정. 임의의 kPa 오차 임계값 대신 **레일 경로가 포화했는가**를 본다.
  //   micro(레일) 명령이 포화 = 레일 밸브를 완전히 열었는데도 요구 유량에 못 미친다
  //                          = 레일만으로는 부족 → macro 를 열 근거가 물리적으로 성립
  // 이 규칙이 "적게 쓰되 반응성 우선"을 동시에 만족한다: 포화하지 않으면 절대 안 열고
  // (최소 사용), 포화하는 즉시 오차 크기와 무관하게 연다 (최대 반응성).
  // 생성기(mode 2)의 축별 부족률 판정과 OR 로 결합된다.
  auto macro_open = [this](float u_micro_req) {
    return macro_allow_.load(std::memory_order_relaxed)
        || u_micro_req >= cfg_.macro_micro_sat_pct;
  };
 
  // Feedforward: 역모델로 필요한 u_pct 계산
  const double Q_req = (double)std::abs(m_dot_pressure + m_dot_volume);

  if (cfg_.is_positive) {
    // 양압 채널: micro=레일→챔버, macro=탱크→챔버, atm=챔버→대기
    u_crack_ = { u_crack((double)P_micro, z_micro_),
                 u_crack((double)P_macro, z_macro_),
                 u_crack((double)P_now,   z_atm_) };
    if ((m_dot_pressure + m_dot_volume) > 0.f) {
      u_mi_req = valve_invert(Q_req, (double)P_micro, (double)P_now, z_micro_)
                 + ki_mi * pos_error_integral_;
      u_at_req = 0.f;
      u_ma_req = macro_open(u_mi_req)
                 ? valve_invert(Q_req, (double)P_macro, (double)P_now, z_macro_)
                   + ki_ma * pos_error_integral_
                 : 0.f;
    } else {
      u_mi_req = 0.f;
      u_ma_req = 0.f;
      u_at_req = valve_invert(Q_req, (double)P_now, (double)P_abs_atm, z_atm_)
                 + ki_at * std::abs(pos_error_integral_);
    }
  } else {
    // 음압 채널: micro=챔버→음압레일, macro=챔버→이젝터, atm=대기→챔버
    u_crack_ = { u_crack((double)P_now,     z_micro_),
                 u_crack((double)P_now,     z_macro_),
                 u_crack((double)P_abs_atm, z_atm_) };
    if ((m_dot_pressure + m_dot_volume) < 0.f) {
      u_mi_req = valve_invert(Q_req, (double)P_now, (double)P_micro, z_micro_)
                 + ki_mi * neg_error_integral_;
      u_at_req = 0.f;
      u_ma_req = macro_open(u_mi_req)
                 ? valve_invert(Q_req, (double)P_now, (double)cfg_.ejector_p_limit, z_macro_)
                   + ki_ma * neg_error_integral_
                 : 0.f;
    } else {
      u_mi_req = 0.f;
      u_ma_req = 0.f;
      u_at_req = valve_invert(Q_req, (double)P_abs_atm, (double)P_now, z_atm_)
                 + ki_at * std::abs(neg_error_integral_);
    }
  }

  // Anti-windup Logic
  float u_mi_req_clamped =  std::clamp(u_mi_req, 0.0f, 100.0f);
  float u_ma_req_clamped =  std::clamp(u_ma_req, 0.0f, 100.0f);
  float u_at_req_clamped =  std::clamp(u_at_req, 0.0f, 100.0f);
  
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

void AcadosMpc::solve(float dt_ms,
                      std::array<uint16_t, MPC_OUT_DIM>& out3,
                      float current_time_sec)
{
  float P_now   = current_P_now_;
  float err     = cfg_.ref_value - P_now;

  // ── 명령 테이퍼 (하드 데드밴드 대체) ─────────────────────────────────────
  // 이 밸브는 **크래킹 임계**가 있다: 솔레노이드 자기력이 스프링/압력을 이겨 스풀을
  // 들어올려야 흐르기 시작하므로, 그 전류(≈50%, 상류 압력에 따라 변동) 아래에서는
  // 유량이 0 이다. 13-variable 모델이 이걸 C_p·Pin 항과 alpha_shape 로 재현한다
  // (실측 ≈50%, 모델은 351 kPa abs 레일에서 ≈52%).
  //
  // 따라서 "개도를 줄인다"는 것은 u 를 0 쪽으로 줄이는 게 아니라 **크래킹 임계 위쪽
  // 여유분**을 줄이는 것이다. u 를 그냥 0 쪽으로 스케일하면 임계 아래로 떨어져
  // 유량이 급절되므로, 연속 테이퍼가 아니라 데드밴드를 다시 만드는 셈이 된다.
  //   u_out = u_crack + (u_req − u_crack) · taper      (u_req > u_crack 일 때)
  // err→0 에서 u_out → u_crack 이고 그 점의 유량이 0 이므로 **유량이 연속**이다.
  // 명령을 u_crack 에서 0 으로 떨어뜨리는 것도 유량 기준으로는 무동작이라 안전하다.
  //
  // 하드 데드밴드를 버린 이유: |err| 로 명령을 잘라내면 그 경계에서 유량이 0 ↔ 큰 값
  // 으로 튀어, 밸브 지연·히스테리시스와 만나면 데드존을 넘나드는 릴레이 진동을 스스로
  // 유발한다. 위 방식은 경계에서 유량이 0 에서 출발하므로 그 기전이 없다.
  // 적분항은 살아 있어 정상상태 오차는 계속 밀어낸다 (수렴만 느려진다).
  const float taper = std::clamp(std::abs(err) / std::max(1e-3f, cfg_.cmd_taper_kpa),
                                 0.0f, 1.0f);

  float P_micro     = current_P_micro_;
  float P_macro     = current_P_macro_;
  float P_macro_neg = current_P_macro_neg_;

  float dt_sec = dt_ms / 1000.0f;
  if (dt_sec <= 0.0001f) dt_sec = cfg_.Ts;

  auto uref_arr = compute_input_reference(P_now, P_micro, P_macro, P_macro_neg, dt_sec, current_time_sec);
  Eigen::RowVector3f u_ref(uref_arr[0], uref_arr[1], uref_arr[2]);

  std::fill(P_ref_.begin(), P_ref_.end(), cfg_.ref_value);

  update_linearization(cfg_.ref_value, u_ref);

  const int Nu = cfg_.n_u * cfg_.NP;
  Pmat_.setZero(Nu, Nu);
  qvec_.setZero(Nu);
  Acon_.setZero(Nu, Nu);
  LL_.setZero(Nu);
  UL_.setZero(Nu);

  build_mpc_qp(A_seq_, B_seq_, P_now, P_ref_, Pmat_, qvec_, Acon_, LL_, UL_);

  for (int i = 0; i < cfg_.NP; ++i) {
      for (int j = 0; j < 3; ++j) { 
          int idx = i * 3 + j;
          float u_current_ref = uref_arr[j]; 
          
          LL_(idx) = std::max(-u_current_ref, cfg_.du_min); 
          UL_(idx) = std::min(100.0f - u_current_ref, cfg_.du_max);
      }
  }

  auto du3 = solve_qp_first_step(Pmat_, qvec_, Acon_, LL_, UL_);

  // 테이퍼는 QP 결과까지 포함한 **최종** 명령에 적용한다. u_ref 만 줄이면 MPC 가 Δu 로
  // 다시 밀어올려 무력화되기 때문이다.
  auto taper_above_crack = [taper](float u, float u_c) {
    return (u <= u_c) ? 0.0f : u_c + (u - u_c) * taper;
  };
  std::array<float,3> u0{
    taper_above_crack(std::clamp(uref_arr[0] + du3[0], 0.0f, 100.0f), u_crack_[0]),
    taper_above_crack(std::clamp(uref_arr[1] + du3[1], 0.0f, 100.0f), u_crack_[1]),
    taper_above_crack(std::clamp(uref_arr[2] + du3[2], 0.0f, 100.0f), u_crack_[2]),
  };
  last_u3_ = u0;

  // 4095 스케일 (100% -> 4095)
  // 40.95 = 4095 / 100
  out3[0] = static_cast<uint16_t>( std::round(u0[0] * 40.95f) ); 
  out3[1] = static_cast<uint16_t>( std::round(u0[2] * 40.95f) ); 
  out3[2] = static_cast<uint16_t>( std::round(u0[1] * 40.95f) ); 
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

  sensor_filter_alpha_ = this->declare_parameter<double>("sensor_filter_alpha", 1.0);

  if (sensor_filter_alpha_ <= 0.0) sensor_filter_alpha_ = 0.01;
  if (sensor_filter_alpha_ > 1.0)  sensor_filter_alpha_ = 1.0;

  RCLCPP_INFO(get_logger(), "Sensor Filter Alpha applied: %.3f", sensor_filter_alpha_);

  // Flat per-board calibration: Sensor_calibration.boards."N".offset/gain
  for (int bid = 1; bid <= NUM_CAN_BOARDS; ++bid) {
    const std::string base = "Sensor_calibration.boards." + std::to_string(bid);
    auto& ch = sensor_.boards[(size_t)(bid - 1)];
    ch.offset = get_param_or<double>(this, base + ".offset", ch.offset);
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
  mpc_.macro_micro_sat_pct = get_param_or<double>(this, "MPC_parameters.macro_micro_sat_pct", 100.0);
  mpc_.cmd_taper_kpa       = get_param_or<double>(this, "MPC_parameters.cmd_taper_kpa",          3.0);
  mpc_.valve_crack_area_frac = get_param_or<double>(this, "MPC_parameters.valve_crack_area_frac", 1e-6);


  default_volume_ml_  = get_param_or<double>(this, "default_volume_ml",    1.0);
  actuator_connected_ = get_param_or<bool>  (this, "actuator_connected",   true);
  tank_volume_pos_ml_ = get_param_or<double>(this, "tank_volume_pos_ml", 750.0);
  tank_volume_neg_ml_ = get_param_or<double>(this, "tank_volume_neg_ml", 400.0);

  vol_ml_.resize(num_total_channels_);
  for (int i = 0; i < num_total_channels_; ++i) {
    if (!actuator_connected_)
      vol_ml_[i] = (i < num_positive_channels_) ? tank_volume_pos_ml_ : tank_volume_neg_ml_;
    else
      vol_ml_[i] = default_volume_ml_;
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

    channel_configs_[i].I_MAX       = get_param_or<double>(this, prefix + "I_MAX",       0.30);
    channel_configs_[i].A_max       = get_param_or<double>(this, prefix + "A_max",       0.2845);
    channel_configs_[i].k_shape     = get_param_or<double>(this, prefix + "k_shape",     33.09);
    channel_configs_[i].C_k         = get_param_or<double>(this, prefix + "C_k",         0.0288);
    channel_configs_[i].C_p         = get_param_or<double>(this, prefix + "C_p",         0.00012);
    channel_configs_[i].C_z         = get_param_or<double>(this, prefix + "C_z",         0.0);
    channel_configs_[i].A_bw        = get_param_or<double>(this, prefix + "A_bw",        260649.5);
    channel_configs_[i].beta_bw     = get_param_or<double>(this, prefix + "beta_bw",     179.0);
    channel_configs_[i].gamma_bw    = get_param_or<double>(this, prefix + "gamma_bw",    0.06);
    channel_configs_[i].alpha_shape = get_param_or<double>(this, prefix + "alpha_shape", 3884.2);
    channel_configs_[i].wn_up       = get_param_or<double>(this, prefix + "wn_up",       40.0);
    channel_configs_[i].zeta_up     = get_param_or<double>(this, prefix + "zeta_up",     1.2);
    channel_configs_[i].wn_down     = get_param_or<double>(this, prefix + "wn_down",     45.0);
    channel_configs_[i].zeta_down   = get_param_or<double>(this, prefix + "zeta_down",   1.0);
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
          const size_t n = std::min(msg->data.size(), encoder_angles_.size());
          for (size_t i = 0; i < n; ++i)
              encoder_angles_[i] = msg->data[i];
      });

  size_t nth = std::max<size_t>(2, std::min<size_t>(
      (size_t)num_total_channels_,
      std::thread::hardware_concurrency()));
  std::vector<int> pins; if (enable_thread_pinning_) for (auto v: cpu_pins_param_) pins.push_back((int)v);
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
  gen_use_ej_meas_  = get_param_or<bool>(this, "PressureRefGen.use_ejector_measurement", true);
  gen_pos_ref_kpa_.assign(num_actuators_, sensor_.kpa_atm());
  gen_neg_ref_kpa_.assign(num_actuators_, sensor_.kpa_atm());
  gen_starve_pos_.assign(num_actuators_, 0.0);
  gen_starve_neg_.assign(num_actuators_, 0.0);
  gen_macro_gate_frac_ = get_param_or<double>(this, "PressureRefGen.macro_gate_frac", 0.02);

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
  }

  {
    PressureRefGen::Params gp;
    gp.N  = num_actuators_;
    gp.dt = std::max(1e-3, gen_period_ms_ / 1000.0);
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
    gp.P_tank_stop  = get_param_or<double>(this, "PressureRefGen.tank_stop_kpa",        450.0) * 1000.0;

    gp.wtrack   = get_param_or<double>(this, "PressureRefGen.weights.track",  100.0);
    gp.w_flow   = get_param_or<double>(this, "PressureRefGen.weights.flow",     0.3);
    gp.w_smooth = get_param_or<double>(this, "PressureRefGen.weights.smooth",   0.5);
    gp.w_tank   = get_param_or<double>(this, "PressureRefGen.weights.tank",    15.0);
    gp.w_eject  = get_param_or<double>(this, "PressureRefGen.weights.eject",   25.0);
    gp.max_iter = get_param_or<int>   (this, "PressureRefGen.sqp_max_iter",     12);

    gp.Cd = get_param_or<double>(this, "PressureRefGen.Cd", 0.8);
    gp.valve_open_eta = get_param_or<double>(this, "PressureRefGen.valve_open_eta", 1.0);
    gp.set_orifices(
      get_param_or<double>(this, "PressureRefGen.orifice_mm.fill",   2.3),
      get_param_or<double>(this, "PressureRefGen.orifice_mm.vent",   4.0),
      get_param_or<double>(this, "PressureRefGen.orifice_mm.boost",  1.6),
      get_param_or<double>(this, "PressureRefGen.orifice_mm.suck",   4.0),
      get_param_or<double>(this, "PressureRefGen.orifice_mm.admit",  4.0),
      get_param_or<double>(this, "PressureRefGen.orifice_mm.eject",  4.0));

    refgen_ = std::make_unique<PressureRefGen>(gp);
    RCLCPP_INFO(get_logger(),
      "PressureRefGen: N=%d, dt=%.0f ms, 정격 P⁺≤%.1f kPa / P⁻≥%.1f kPa (gauge), "
      "Cd=%.2f eta=%.2f, F_max=%.1f N → τ_max=%.2f N·m",
      gp.N, gp.dt * 1e3, gp.Pch_pos_max / 1e3, gp.Pch_neg_min / 1e3,
      gp.Cd, gp.valve_open_eta,
      gp.Pch_pos_max * A_m2 + std::abs(gp.Pch_neg_min) * A_m2,
      (gp.Pch_pos_max + std::abs(gp.Pch_neg_min)) * A_m2 * reel_radius_mm_ * 1e-3);

    RCLCPP_INFO(get_logger(), "펌프 능력 테이블 계산 중...");
    const auto t0 = std::chrono::steady_clock::now();
    refgen_->build_pump_table();
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
  active_channels_.clear();
  for (int i = 0; i < num_actuators_; ++i) {
    active_channels_.insert(i);                              // 양압 gid 0..N-1
    active_channels_.insert(num_positive_channels_ + i);    // 음압 gid num_pos..num_pos+N-1
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

      cfg.I_MAX       = (float)channel_configs_[gid].I_MAX;
      cfg.A_max       = (float)channel_configs_[gid].A_max;
      cfg.k_shape     = (float)channel_configs_[gid].k_shape;
      cfg.C_k         = (float)channel_configs_[gid].C_k;
      cfg.C_p         = (float)channel_configs_[gid].C_p;
      cfg.C_z         = (float)channel_configs_[gid].C_z;
      cfg.A_bw        = (float)channel_configs_[gid].A_bw;
      cfg.beta_bw     = (float)channel_configs_[gid].beta_bw;
      cfg.gamma_bw    = (float)channel_configs_[gid].gamma_bw;
      cfg.alpha_shape = (float)channel_configs_[gid].alpha_shape;
      cfg.wn_up       = (float)channel_configs_[gid].wn_up;
      cfg.zeta_up     = (float)channel_configs_[gid].zeta_up;
      cfg.wn_down     = (float)channel_configs_[gid].wn_down;
      cfg.zeta_down   = (float)channel_configs_[gid].zeta_down;


      cfg.ref_value = 101.325f;
      cfg.du_min = -30.f; cfg.du_max = +30.f;
      cfg.u_abs_min = 0.f; cfg.u_abs_max = 100.f;

      cfg.volume_m3 = ml_to_m3(vol_ml_[gid]);

      cfg.ejector_k       = (float)mpc_.ejector_k;
      cfg.ejector_p_limit = (float)mpc_.ejector_p_limit;

      cfg.leakage_u_pos = (float)mpc_.leakage_u_pos;
      cfg.leakage_u_neg = (float)mpc_.leakage_u_neg;

      cfg.target_time_constant = (float)mpc_.target_tc;
      cfg.macro_micro_sat_pct  = (float)mpc_.macro_micro_sat_pct;
      cfg.cmd_taper_kpa        = (float)mpc_.cmd_taper_kpa;
      cfg.valve_crack_area_frac = (float)mpc_.valve_crack_area_frac;

      auto mpc_obj = std::make_unique<AcadosMpc>(cfg);
      int nv = cfg.n_u * cfg.NP; 
      int nc = 0; 
      auto qp_solver = std::make_shared<QP>(nv, nc);
      mpc_obj->set_qp_solver(qp_solver);

      mpcs_.emplace_back(std::move(mpc_obj));
  }

  RCLCPP_INFO(get_logger(), "Initialized %zu MPC controllers based on active_mpc_channels parameter.", mpcs_.size());

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

void Controller::on_timer() {
  auto now = std::chrono::steady_clock::now();
  elapsed_time_sec_ = std::chrono::duration<double>(now - start_time_).count();

  std::array<uint16_t, NUM_CAN_BOARDS> snap_sensors;
  {
    std::lock_guard<std::mutex> lk(sensors_mtx_);
    snap_sensors = sensors_raw_;
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
        if (sensor_zero_cnt_[i] > 0)
          sensor_.boards[i].offset = sensor_zero_sum_[i] / sensor_zero_cnt_[i];
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

    // 엔코더 보드 (17~22): 반전증폭 역산(1~5V→3.3V~0V) 후 orig_mV 기준 각도 계산
    if (bid >= 17 && bid <= 22) {
      double adc_mv = std::clamp((double)snap_sensors[idx] * (3300.0 / 4095.0), 0.0, 3300.0);
      double orig_mv = (4125.0 - adc_mv) / 0.825;
      const auto& ec = sensor_.boards[(size_t)idx];
      double angle_deg = (orig_mv - ec.offset) * ec.gain;
      if (!filter_initialized_) filt_state_[idx] = angle_deg;
      filt_state_[idx] = sensor_filter_alpha_ * angle_deg + (1.0 - sensor_filter_alpha_) * filt_state_[idx];
      filt_out_[idx]   = filt_state_[idx];
      continue;
    }

    // 라인 압력 보드 or 활성 채널 보드만 처리
    bool is_line_board = (bid == P_pos_board_id_ || bid == P_neg_board_id_ || bid == P_macro_board_id_ || bid == P_macro_neg_board_id_);
    int gid = bid - channel_board_offset_;
    if (!is_line_board && (gid < 0 || gid >= num_total_channels_ || active_channels_.count(gid) == 0)) {
      filt_out_[idx] = sensor_.kpa_atm();
      continue;
    }
    double raw_kpa = sensor_.kpa(bid, snap_sensors[idx]);
    if (!filter_initialized_) filt_state_[idx] = raw_kpa;
    filt_state_[idx] = sensor_filter_alpha_ * raw_kpa + (1.0 - sensor_filter_alpha_) * filt_state_[idx];
    filt_out_[idx]   = filt_state_[idx];
  }
  filter_initialized_ = true;

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
  if (control_mode_ == 1) {
    const double dt_sec = std::max(1e-6, (double)period_ms_ / 1000.0);
    run_position_control(dt_sec);
  } else if (control_mode_ == 2) {
    const double dt_sec = std::max(1e-6, (double)period_ms_ / 1000.0);
    run_optimized_pressure_ref(dt_sec);
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

          vol_ml_[i] = tank_volume_pos_ml_
              + A * std::max(0.0, vol_offset_pos_mm_ + x_mm) / 1000.0;

          if (neg_gid < num_total_channels_ && active_channels_.count(neg_gid))
              vol_ml_[neg_gid] = tank_volume_neg_ml_
                  + A * std::max(0.0, vol_offset_neg_mm_ - x_mm) / 1000.0;
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

      std::array<uint16_t, MPC_OUT_DIM> u3{};
      m->solve(static_cast<float>(period_ms_), u3, static_cast<float>(elapsed_time_sec_));

      const int pwm_base = brd_idx * PWM_PER_BOARD;
      zoh_[pwm_base + 0] = u3[0];
      zoh_[pwm_base + 1] = u3[1];
      zoh_[pwm_base + 2] = u3[2];
    });
  }

  pool_->run_batch_and_wait(tasks);
  
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

  const double dt = std::max(1e-6, (double)period_ms_ / 1000.0);

  // -------------------------------------------------------------
  // [양압 라인 PID] (Positive Line)
  // -------------------------------------------------------------
  {
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
  {
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
      angle_ref = pos_tcp_received_ ? target_angle_deg_[(size_t)a] : angle;
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
    const double error = angle_ref - angle;
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
            - cfg.m1.kd * vel;   // 미분: 측정값 미분 (setpoint kick 방지)
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
      angle_ref = pos_tcp_received_ ? target_angle_deg_[(size_t)a] : angle;
    }

    if (!state.initialized) {
      state.prev_angle = angle; state.vel_filt = 0.0; state.initialized = true;
      tau_integ_[(size_t)a] = 0.0;
    }
    const double vel_raw = (angle - state.prev_angle) / dt_sec;
    state.vel_filt = cfg.vel_filter_alpha * vel_raw + (1.0 - cfg.vel_filter_alpha) * state.vel_filt;
    state.prev_angle = angle;
    const double vel = state.vel_filt;

    const double err = angle_ref - angle;

    // 적분 (부호 반전 시 리셋 + 클램프)
    double& I = tau_integ_[(size_t)a];
    if ((err > 0.0 && I < 0.0) || (err < 0.0 && I > 0.0)) I = 0.0;
    if (actuator_connected_) I += err * dt_sec;
    const double I_lim = (std::abs(tp.ki) > 1e-12) ? tp.integ_limit_nm / tp.ki : 0.0;
    I = std::clamp(I, -I_lim, I_lim);

    const double tau_pid = actuator_connected_
        ? (tp.kp * err + tp.ki * I - tp.kd * vel) : 0.0;

    // 중력 피드포워드 — 이제 곱셈 없이 그대로 토크다
    const double ff_angle = actuator_connected_ ? angle : angle_ref;
    const double tau_grav = cfg.mass_kg * 9.81 * cfg.link_length_m
                          * std::sin(ff_angle * M_PI / 180.0);

    const double tau_fric = (actuator_connected_ && std::abs(err) > 0.3)
        ? tp.friction_nm * (err > 0.0 ? 1.0 : -1.0) : 0.0;

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

    for (int a = 0; a < N; ++a) {
      gen_pos_ref_kpa_[(size_t)a] = to_abs_kpa(r.P_pos_ref[(size_t)a]);
      gen_neg_ref_kpa_[(size_t)a] = to_abs_kpa(r.P_neg_ref[(size_t)a]);
    }
    gen_rail_pos_sp_kpa_ = to_abs_kpa(r.rail_pos_sp);
    gen_rail_neg_sp_kpa_ = to_abs_kpa(r.rail_neg_sp);
    gen_has_result_ = true;

    // ── macro 게이트: 생성기의 축별 유량 부족률로 구동 ────────────────────
    // 원래 철학(transient 에서 유량이 부족해 목표에 빠르게 못 갈 때 부스팅)을
    // 오차 임계값 대신 "레일이 이번 스텝 수요를 감당하는가"로 판정한다. 슬루 박스가
    // 이미 부스트/이젝터 능력을 포함해 레퍼런스를 만들었으므로, 부족률 > 0 은
    // "그 능력을 쓸 계획"이라는 뜻이고 이렇게 하면 계획과 밸브 동작이 일치한다.
    // MPC 쪽 micro 포화 판정과 OR 로 결합된다 (AcadosMpc::set_macro_allow 주석 참조).
    for (int a = 0; a < N; ++a) {
      const auto& cfg = pos_ctrl_cfg_[(size_t)a];
      const double sp = r.starve_pos[(size_t)a], sn = r.starve_neg[(size_t)a];
      if (auto* m = mpc_for_gid(cfg.pos_gid)) m->set_macro_allow(sp > gen_macro_gate_frac_);
      if (auto* m = mpc_for_gid(cfg.neg_gid)) m->set_macro_allow(sn > gen_macro_gate_frac_);
      gen_starve_pos_[(size_t)a] = sp * 100.0;
      gen_starve_neg_[(size_t)a] = sn * 100.0;
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
        r.starve_pos[0] > gen_macro_gate_frac_ ? "B" : "-",
        r.starve_neg[0] > gen_macro_gate_frac_ ? "E" : "-",
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