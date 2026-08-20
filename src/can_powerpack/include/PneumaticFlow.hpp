#pragma once

// ============================================================================
// PneumaticFlow — 압축성 오리피스 유량 공용 함수
// ============================================================================
// pressure_reference_optimizer.m 의 orifice_phi / valve_phys_kgps /
// valve_capacity 를 그대로 옮긴 것이다. 단위는 MATLAB 원본과 동일하게
// **절대압 [Pa]** 을 받는다 (게이지가 아니다 — 호출부에서 P_atm 을 더해서 넘긴다).
//
// 주의: 이 파일은 신규 코드(PressureRefGen)만 사용한다. Controller.cpp 의
// 13-variable 밸브 모델과 VirtualPowerpack/PneumaticSim 에 있는 기존 get_phi
// 사본들은 이미 튜닝된 상태이므로 이번에는 통합하지 않는다.
// ============================================================================

#include <cmath>

namespace pneu {

// 공기 물성 (MATLAB build_params 와 동일)
constexpr double R_AIR   = 287.0;      // 이상기체상수 [J/(kg·K)]
constexpr double KAPPA   = 1.4;        // 비열비
constexpr double P_ATM   = 101325.0;   // 대기압 [Pa]
constexpr double T_CH    = 293.15;     // 챔버 온도 [K]
constexpr double T_PIS   = 323.15;     // 피스톤 온도 [K] — 펌프는 더 뜨겁다
constexpr double RHO0    = 1.204;      // 표준 공기밀도 [kg/m³] (LPM 환산용)

// 표준 LPM → kg/s.  = 1.204 / 60 / 1000
// 주의: repo 의 13-variable 밸브 모델이 쓰는 LPM_TO_KGPS(2.155e-4) 는 그 모델을
// fit 할 때 나온 **경험적 단위**이고 표준 LPM 이 아니다 (10.7배 차이).
// 이젝터 카탈로그 값(흡입 100 LPM / 소비 57 LPM)이나 탱크 저장량처럼 실제 물리
// 단위로 주어진 값은 반드시 이 상수로 환산해야 한다.
constexpr double STD_LPM_TO_KGPS = RHO0 / 60.0 / 1000.0;

// ── 압축성 오리피스 유량함수 Φ (논문 식 3) ────────────────────────────────
// Pin/Pout: 절대압 [Pa]. 역류(Pr > 1)는 0 을 돌려준다.
inline double orifice_phi(double Pin, double Pout, double kappa = KAPPA)
{
  if (Pin <= 0.0 || Pout <= 0.0) return 0.0;
  const double Pr = Pout / Pin;
  if (Pr > 1.0) return 0.0;
  const double Pcr = std::pow(2.0 / (kappa + 1.0), kappa / (kappa - 1.0));
  if (Pr <= Pcr)   // 초킹
    return std::sqrt(kappa * std::pow(2.0 / (kappa + 1.0), (kappa + 1.0) / (kappa - 1.0)));
  return std::sqrt(2.0 * kappa / (kappa - 1.0))
       * std::sqrt(std::max(0.0, std::pow(Pr, 2.0 / kappa) - std::pow(Pr, (kappa + 1.0) / kappa)));
}

// ── 물리 오리피스 질량유량 [kg/s] ─────────────────────────────────────────
// A_eff = Cd · πd²/4 [m²],  Pup/Pdn 절대압 [Pa]
// 슬루 한계와 공급원 수지에 쓰인다.
inline double valve_phys_kgps(double Pup_abs, double Pdn_abs, double A_eff,
                              double T = T_CH, double kappa = KAPPA)
{
  return A_eff * Pup_abs / std::sqrt(R_AIR * T) * orifice_phi(Pup_abs, Pdn_abs, kappa);
}

// ── 완전개방 기준 "가용유량" (상대 단위) ──────────────────────────────────
// MATLAB valve_capacity: Q = A_max · P_up · Φ. 13-variable 밸브모델의 A_max 를
// 그대로 쓰는 경험적 상대단위이고, 목적함수 J2 정규화에만 사용한다.
inline double valve_capacity(double Pup_abs, double Pdn_abs, double A_max,
                             double kappa = KAPPA)
{
  return A_max * Pup_abs * orifice_phi(Pup_abs, Pdn_abs, kappa);
}

}  // namespace pneu
