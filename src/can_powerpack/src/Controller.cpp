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
                "Listening for pressure refs on port %d  [double pos_kpa, double neg_kpa]",
                cfg_.port);

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

        constexpr size_t MSG = 2 * sizeof(double);
        uint8_t buf[MSG];
        bool ok = true;

        while (!stop_.load() && ok) {
            size_t total = 0;
            while (total < MSG && !stop_.load()) {
                ssize_t n = ::recv(cfd, buf + total, MSG - total, 0);
                if (n == 0) { ok = false; break; }
                if (n < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
                    ok = false; break;
                }
                total += (size_t)n;
            }
            if (!ok || total < MSG) break;

            double pos_kpa, neg_kpa;
            std::memcpy(&pos_kpa, buf,                  sizeof(double));
            std::memcpy(&neg_kpa, buf + sizeof(double),  sizeof(double));
            cb_(pos_kpa, neg_kpa);
        }

        ::close(cfd); client_fd_.store(-1);
        RCLCPP_INFO(rclcpp::get_logger("RefTcpServer"), "Client disconnected.");
    }
    ::close(sfd); server_fd_.store(-1);
#endif
}

template <typename T>
static T get_param_or(rclcpp::Node* node, const std::string& name, const T& defv) {
  try {
    return node->declare_parameter<T>(name, defv);
  } catch (...) {
    return defv;
  }
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

  // Inverse valve model: given required Q [LPM], P_in, P_out, z → u_pct [0,100]
  auto valve_invert = [&](double Q_req, double Pin, double Pout, double z_val) -> float {
    if (Q_req <= 0.0) return 0.0f;
    const double phi = get_phi_ff(Pin, Pout);
    if (phi < 1e-9) return 0.0f;
    const double Area_req = Q_req / (Pin * phi);
    const double A_max_d  = (double)cfg_.A_max;
    const double alpha_d  = (double)cfg_.alpha_shape;
    if (Area_req >= A_max_d) return 100.0f;
    const double sigma = std::pow(std::clamp(Area_req / A_max_d, 1e-9, 1.0-1e-9), 1.0/alpha_d);
    const double F_req = std::log(sigma / (1.0 - sigma)) / (double)cfg_.k_shape;
    const double I_req = F_req - (double)cfg_.C_z * z_val - (double)cfg_.C_p * Pin + (double)cfg_.C_k;
    return (float)std::clamp(I_req / (double)cfg_.I_MAX * 100.0, 0.0, 100.0);
  };

  // [중요] 에러 계산을 최상단으로 이동
  const float Pref = cfg_.ref_value;
  float err = Pref - P_now;
  
  // =========================================================================
  // [수정] Deadband (Threshold) Check
  // 에러의 절대값이 설정된 threshold(예: 3.0)보다 작으면 무조건 0 리턴
  // =========================================================================
  if (std::abs(err) < cfg_.actuating_threshold) {
      // 데드밴드 진입 시 적분항 누적 방지 (선택 사항: 0으로 리셋하거나 현상유지)
      // 여기서는 튀는 것을 방지하기 위해 리셋하지 않고 통과하거나, 
      // 필요하다면 아래 주석을 해제하세요.
      // pos_error_integral_ = 0.0f;
      // neg_error_integral_ = 0.0f;
      
      return {0.0f, 0.0f, 0.0f}; // 밸브 닫음
  }

  // --- 기존 로직 수행 ---

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
 
  // Feedforward: 역모델로 필요한 u_pct 계산
  const double Q_req = (double)std::abs(m_dot_pressure + m_dot_volume);

  if (cfg_.is_positive) {
    if ((m_dot_pressure + m_dot_volume) > 0.f) {
      u_mi_req = valve_invert(Q_req, (double)P_micro, (double)P_now, z_micro_)
                 + ki_mi * pos_error_integral_;
      u_at_req = 0.f;
      u_ma_req = (std::abs(err) >= cfg_.macro_threshold)
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
    if ((m_dot_pressure + m_dot_volume) < 0.f) {
      u_mi_req = valve_invert(Q_req, (double)P_now, (double)P_micro, z_micro_)
                 + ki_mi * neg_error_integral_;
      u_at_req = 0.f;
      u_ma_req = (std::abs(err) >= cfg_.macro_threshold)
                 ? valve_invert(Q_req, (double)P_now, (double)P_macro_neg, z_macro_)
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

  // QP Solver (A_con 미사용 — 단순 바운드 구조)
  bool success = qp_->solve(P, q, LL, UL, solution_);

  if (!success) {
      if (qp_fail_count_++ == 0) {
          RCLCPP_WARN(rclcpp::get_logger("AcadosMpc"),
                      "QP solve failed (gid=%d), outputting zero.", cfg_.global_id);
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
  
  // =========================================================================
  // [수정] Deadband (Threshold) Check - 최상단 배치
  // 이 부분이 없으면 Feedforward가 0이어도 MPC가 미세한 오차를 줄이려고 동작함
  // =========================================================================
  float err = cfg_.ref_value - P_now;
  if (std::abs(err) < cfg_.actuating_threshold) {
      // 오차 범위 내에서는 계산하지 않고 0 출력 후 종료
      out3[0] = 0;
      out3[1] = 0;
      out3[2] = 0;
      
      // Last u 업데이트 (0으로)
      last_u3_ = {0.0f, 0.0f, 0.0f};
      return; 
  }

  // --- 기존 로직 수행 ---
  
  float P_micro     = current_P_micro_;
  float P_macro     = current_P_macro_;
  float P_macro_neg = current_P_macro_neg_;

  float dt_sec = dt_ms / 1000.0f;
  if (dt_sec <= 0.0001f) dt_sec = cfg_.Ts;

  // 위에서 compute_input_reference를 수정했으므로,
  // 여기서도 threshold 조건이면 {0,0,0}이 리턴됨
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

  std::array<float,3> u0{
    std::clamp(uref_arr[0] + du3[0], 0.0f, 100.0f),
    std::clamp(uref_arr[1] + du3[1], 0.0f, 100.0f),
    std::clamp(uref_arr[2] + du3[2], 0.0f, 100.0f),
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
  mpc_.macro_threshold = get_param_or<double>(this, "MPC_parameters.macro_threshold", 30.0);
  mpc_.actuating_threshold = get_param_or<double>(this, "MPC_parameters.actuating_threshold", 5.0);


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

  sub_volumes_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "actuator/volumes_ml", 10, std::bind(&Controller::on_volume, this, _1));

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

  ref_server_cfg_.enable  = get_param_or<bool>(this, "RefTcpServer.enable",  false);
  ref_server_cfg_.port    = get_param_or<int> (this, "RefTcpServer.port",    2293);
  ref_server_cfg_.pos_gid = get_param_or<int> (this, "RefTcpServer.pos_gid", 0);
  ref_server_cfg_.neg_gid = get_param_or<int> (this, "RefTcpServer.neg_gid", num_positive_channels_);

  if (ref_server_cfg_.enable) {
    ref_server_ = std::make_unique<RefTcpServer>(
      ref_server_cfg_,
      [this](double pos_kpa, double neg_kpa) {
        std::lock_guard<std::mutex> lk(mpc_ref_mtx_);
        const int pg = ref_server_cfg_.pos_gid;
        const int ng = ref_server_cfg_.neg_gid;
        if (pg >= 0 && pg < (int)mpc_ref_kpa_.size()) mpc_ref_kpa_[pg] = pos_kpa;
        if (ng >= 0 && ng < (int)mpc_ref_kpa_.size()) mpc_ref_kpa_[ng] = neg_kpa;
      }
    );
    RCLCPP_INFO(get_logger(), "RefTcpServer: enabled on port %d (pos_gid=%d, neg_gid=%d)",
                ref_server_cfg_.port, ref_server_cfg_.pos_gid, ref_server_cfg_.neg_gid);
  }

  filt_state_.assign(NUM_CAN_BOARDS, 101.325);

  zero_calib_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/zero_calibration",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr req,
           std_srvs::srv::Trigger::Response::SharedPtr res) {
      on_zero_calibration(req, res);
    });

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
  int num_actuators = get_param_or<int>(this, "num_actuators", 1);
  active_channels_.clear();
  for (int i = 0; i < num_actuators; ++i) {
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
      cfg.macro_threshold      = (float)mpc_.macro_threshold;
      cfg.actuating_threshold  = (float)mpc_.actuating_threshold;

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

  {
    std::lock_guard<std::mutex> lk(mpc_ref_mtx_);
    ref_snapshot_ = mpc_ref_kpa_;
  }
  if (pub_mpc_refs_) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(ref_snapshot_.begin(), ref_snapshot_.end());
    pub_mpc_refs_->publish(msg);
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

  publish_cmds();

  for(int i = 0; i < num_total_channels_; ++i) {
      prev_vol_m3_[i] = final_active_vols_ml_[i] * 1.0e-6;
  }

  ++tick_;
}

void Controller::inner_loop_1khz(float /*dt_ms*/) {
  std::fill(inner_.begin(), inner_.end(), 0);
}

void Controller::publish_cmds() {
  std_msgs::msg::UInt16MultiArray m;
  m.data.assign(cmds_.begin(), cmds_.end());
  pub_pwm_cmd_->publish(m);
}