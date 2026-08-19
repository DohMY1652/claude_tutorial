%% pressure_reference_optimizer.m
% =========================================================================
%  최적화 기반 압력 레퍼런스(P_ref) 생성 - 양/음압 동시 사용 + 탱크 부스터
%  [실시간 대응] pressure_reference_init(1회) + pressure_reference_step(매 틱)
% -------------------------------------------------------------------------
%  공급원 3개 (모두 동적 상태):
%    (1) 양압레일  : 피스톤펌프가 채움. 셋포인트는 고정이 아니라 매 스텝 자동배분.
%    (2) 음압레일  : 같은 펌프가 비움(흡입=토출, 직렬로 묶임 -> 양/음 상한 트레이드오프).
%    (3) 압축탱크  : 700kPa 부스터. 컴프레서 없음(외부충전 후 소진) -> 처지면 회복X.
%                    레일이 처져 못 따라갈 때만 잠깐 쓰는 예비원 -> 사용 최소화가 목표.
%
%  [레일 셋포인트 자동배분] decide_rail_setpoint:
%    핵심 물리: 채널로 가는 유량은 '레일 압력'으로 정해짐(펌프 순환 여부 무관).
%      -> 목표는 레일압을 최대한 높게 유지. 릴리프는 셋포인트 이상만 최소 방출.
%    단, 흡입=토출이라 양압을 높이면 음압이 억눌림(트레이드오프).
%    이 시스템은 한 방향(+힘)만 냄. 음압도 +힘 보조 -> '음압수요' 개념 없음.
%      -> 프리뷰(미래수요)의 '총 크기'로 양·음압 레일을 함께 조절:
%         총수요 크면 양쪽 다 깊게(여력 확보), 작으면 여유만(절약).
%    각 상한 = 그 배분에서 펌프가 물리적으로 닿는 최대(cap 테이블, init에서 사전계산).
%
%  결정변수: 각 액추에이터 목표압력 {P_pos,i , P_neg,i}  (공급원 압력은 상태, 결정변수 아님)
%  채널 상한: 액추에이터 정격  P_pos <= 175kPa,  P_neg >= -84kPa(이젝터 한계)
%  ※ 액추에이터 1개 = 양압채널 1 + 음압채널 1.  N=액추에이터 수(=6). 압력변수는 2N개.
%
%  밸브 구성 (총 6N+3):
%    양압채널 3개: 양압레일->채널(2.3mm) / 채널->대기(4mm) / 탱크->채널(1.6mm,부스트)
%    음압채널 3개: 채널->음압레일(4mm) / 대기->채널(4mm,완화) / 채널->이젝터(4mm,심화)
%    => 액추에이터당 6개 x N + 릴리프 2개(양/음압레일<->대기 1.6mm) + 이젝터 구동 1개
%
%  목적함수(가중합, 채널변수만):
%     J0)    토크 소프트 추종:  tau ~= P_pos*A_pos - P_neg*A_neg  (양+음압 힘 합산)
%     J2)    총 가용유량 최대화(반응 여지 확보)
%     Jfast) '빠른 쪽 우선' 분해: 힘을 그 순간 압력변화가 빠른 채널(유량 여유 큰 쪽)에
%            우선 배분. 느린 채널로 힘을 내면 슬루 역수만큼 페널티.
%     J4)    압력 급변 억제(평활)
%     Jtank ) 탱크(부스트) 사용 최소화  - "레일이 못 대는 초과분"에만 부과
%     Jeject) 이젝터 사용 최소화        - 탱크를 더 빨리 소모하므로 가장 강한 페널티
%             (둘 다 탱크 잔량이 줄수록 페널티 증폭 -> 남은 자원 더 아낌)
%
%  채널 스텝당 이동한도(슬루)는 그 순간 공급원-채널 차압에서 나오는 실제 밸브
%  물리유량으로 계산 (양압상승=레일+탱크 합, 음압심화=음압레일+이젝터 합).
%
% -------------------------------------------------------------------------
%  [밸브 물리유량 kg/s]: 오리피스식 (경로별 실제 지름, Cd=0.8 공통) -> 슬루/수지
%  [밸브 상대유량]     : 업로드 valve_step_v2 포팅 (목적함수 J2/J3 정규화용)
%  [펌프 유량식]       : 논문(RA-L) 식(1)~(3) 포팅, 2D 테이블 사전계산
%    ※ 이 펌프는 Ppos=500k & Pneg=-90k "동시" 유지 불가(같은 피스톤). 양압레일 상한 ~300k.
%  [탱크-이젝터 결합]  : 이젝터가 탱크공기 소비(57/100 환산)까지 탱크 수지에 반영.
%
%  내부 단위: 압력 [Pa](게이지), 면적 [m^2], 질량유량 [kg/s]
%  필요: Optimization Toolbox (fmincon)
%
% -------------------------------------------------------------------------
%  [2차 검토 반영사항]  (해설문서 Q&A 대응)
%   Q1  챔버 동역학을 등온->단열(n_ch=kappa)로 수정. 에너지식에서 엄밀 유도.
%       슬루 1.43배 증가, 추종오차 0.380->0.274N, 불필요한 탱크소모 제거.
%       레일/탱크는 느리므로 등온(n_rail=1) 유지.
%   Q2  펌프 주기평균: 3000rpm -> 1회전=20ms = 제어주기와 일치(맥동 2회/주기).
%       레일 리플 0.8~2.6kPa. 제어주기를 20ms보다 줄이면 순간유량 모델 필요.
%       펌프속도가 부하에 따라 변하면 (Ppos,Pneg,omega) 3D 테이블로 확장 권장.
%   Q5  J_fast 제거(w_fast=0). 포화 시 최적 코너에서 해를 끌어내려 추종을 해침.
%       w_fast=0에서도 음압은 힘의 33%를 담당(방치되지 않음).
%   Q6  탱크 scarcity(잔량 반비례) 증폭 제거. 성능이 서서히 저하되어 하위
%       MPC/RL에 비정상 플랜트가 되고 남은 자원도 못 씀. 대신 P_tank_stop
%       임계값 + usage.tank_low 플래그로 교체/정지를 명시적으로 판단.
%   Q9  프리뷰 H: 25(0.5s) -> 100(2s). 레일 시상수와 맞춤. 개선 +9%->+20%.
%   Q10 이젝터: 정압 싱크(-84kPa) -> 진공도-유량 특성곡선. 정압 가정은 채널
%       0kPa에서 흡입 120LPM(정격 100의 1.2배)을 요구하고 도달진공을 과대평가.
%       측정값이 있으면 sys.P_ej_meas에 넣으면 그대로 사용(가장 정확).
%   Q11 피스톤 운동에 의한 체적변화 항 추가(sys.dVdt_pos/neg, 기본 0).
%       5kg/200mm 조건에서는 힘감쇠(21.5ms)가 부하운동보다 빨라 영향 미미.
%   Q7  레일 부피 민감도: 1L/200mL/50mL는 정상, 10mL에서 붕괴(오차 27배).
%       레일이 챔버보다 작아지면 시간스케일 분리가 역전되어 2계층 설계가 무효.
% =========================================================================

function pressure_reference_optimizer
% 데모 실행 스크립트. 실시간 사용 시에는 아래 API 2개만 있으면 됩니다:
%   sys        = pressure_reference_init();                      % 1회, 오프라인에서 미리 실행
%   [ch,rail]  = pressure_reference_step(tau_k, ch, rail, sys);   % 매 제어 틱마다 호출
% 이 함수는 그 두 API를 알려진 데모 궤적에 대해 루프로 돌려 결과를 검증/시각화합니다.
clc; close all;

sys = pressure_reference_init();

%% ---------------- 목표 토크 궤적: 6액추에이터, 임의 크기·임의 타이밍 (데모용) ----------------
% 각 액추에이터가 서로 다른 시각에 서로 다른 크기의 힘을 원함(사다리꼴 펄스).
% ※ 이 시스템은 각 액추에이터가 '한 방향(+힘)'만 낼 수 있음 -> 목표는 항상 tau>=0.
%   양압과 음압은 서로 반대가 아니라 '같은 +힘을 함께' 만듦:
%   tau = P_pos*A_pos - P_neg*A_neg  (P_neg는 게이지 음수라 -P_neg*A_neg가 +힘에 기여).
% 실전에선 이 배열 대신 그때그때 들어오는 실제 목표 토크(tau_k>=0)를 step에 넘기면 됩니다.
% 현실적 부하로 설정(모든 액추에이터가 동시에 최대토크를 내는 상황은 드묾).
t  = (0:sys.dt:8)';   Nt = numel(t);
specs = [ 0.5,  60, 0.30;    % [시작시각 s, 목표크기 N(>=0), 펄스폭 s]  - 작고 느림
          1.5, 150, 0.40;    % 중간, 빠름
          2.2, 120, 1.50;    % 중간, 느림
          3.0, 130, 0.25;    % 중간, 매우 빠름 -> 순간 유량요구 큼
          4.5,  90, 0.90;    % 작고 느림
          5.0, 140, 0.35 ];  % 중간, 빠름
tau_traj = zeros(Nt, sys.N);
for c = 1:sys.N
    t0=specs(c,1); mag=specs(c,2); w=specs(c,3);
    ramp_up   = min(max((t-t0)/w, 0), 1);
    ramp_down = min(max((t0+2*w-t)/w, 0), 1);
    tau_traj(:,c) = mag .* ramp_up .* ramp_down;   % 사다리꼴 펄스
end

%% ---------------- 초기 상태 ----------------
ch   = zeros(2*sys.N,1);                              % [Ppos(1..N); Pneg(1..N)] 게이지 Pa
rail = [200e3; -50e3; sys.P_tank_sp];  % [양압레일; 음압레일; 탱크] 초기값(중립, 곧 수요따라 적응)

%% ---------------- 로그 ----------------
L.Ppos=zeros(Nt,sys.N); L.Pneg=zeros(Nt,sys.N);       % next(출력) 채널압
L.Ppp=zeros(Nt,1); L.Pnp=zeros(Nt,1); L.Ptk=zeros(Nt,1);  % 공급원 상태(양압레일/음압레일/탱크)
L.PppSP=zeros(Nt,1); L.PnpSP=zeros(Nt,1);             % 레일 셋포인트(수요따라 적응)
L.Qpos=zeros(Nt,sys.N); L.Qneg=zeros(Nt,sys.N);
L.PposCur=zeros(Nt,sys.N); L.PnegCur=zeros(Nt,sys.N); % current(입력) 채널압
L.tankdraw=zeros(Nt,1); L.boost=zeros(Nt,1); L.eject=zeros(Nt,1);
L.cpu=zeros(Nt,1);

%% ---------------- 매 제어 틱 호출 (실시간에서 그대로 쓸 부분) ----------------
% 실시간에선 [ch,rail]=pressure_reference_step(tau_k, ch, rail, sys, tau_preview) 한 줄.
% 여기선 공급원 사용량(usage)까지 로깅하려고 내부 함수들을 펼쳐서 호출합니다.
Hprev = sys.Hpreview;
for k = 1:Nt
    tau_k = tau_traj(k,:).';
    % 프리뷰 창(미래 Hprev 스텝의 목표 토크). 실시간에선 상위 계획기가 제공.
    kf = min(k+Hprev-1, Nt);
    tau_preview = tau_traj(k:kf, :).';          % N x (창길이)
    ch_cur = ch; rail_cur = rail;
    tic;
    rail_sp = decide_rail_setpoint(tau_preview, sys);            % 레일 셋포인트 배분
    ch = optimize_channels(tau_k, ch_cur, rail_cur, sys);        % 채널 압력 최적화
    [rail, usage] = update_sources(rail_cur, ch_cur, ch, rail_sp, sys);  % 공급원 갱신
    L.cpu(k) = toc;

    Ppos=ch(1:sys.N); Pneg=ch(sys.N+1:2*sys.N);
    [Qpos,Qneg] = channel_flows(ch, rail, sys);
    L.PposCur(k,:)=ch_cur(1:sys.N).'; L.PnegCur(k,:)=ch_cur(sys.N+1:2*sys.N).';
    L.Ppos(k,:)=Ppos.'; L.Pneg(k,:)=Pneg.';
    L.Ppp(k)=rail(1); L.Pnp(k)=rail(2); L.Ptk(k)=rail(3);
    L.PppSP(k)=rail_sp(1); L.PnpSP(k)=rail_sp(2);
    L.Qpos(k,:)=Qpos.'; L.Qneg(k,:)=Qneg.';
    L.boost(k)=usage.boost; L.eject(k)=usage.eject; L.tankdraw(k)=usage.tank_draw;
end
fprintf('1-step 계산 시간: 평균 %.2f ms / 최대 %.2f ms  (실시간 주기 %.1fms 대비)\n', ...
    1e3*mean(L.cpu), 1e3*max(L.cpu), 1e3*sys.dt);
fprintf('탱크: %.0f -> %.0f kPa (사용 %.0f kPa),  누적 부스트 %.3g g, 이젝터 %.3g g\n', ...
    sys.P_tank_sp/1e3, L.Ptk(end)/1e3, (sys.P_tank_sp-L.Ptk(end))/1e3, ...
    1e3*sum(L.boost)*sys.dt, 1e3*sum(L.eject)*sys.dt);

%% ---------------- 결과 표 & CSV (전체 N채널) ----------------
% 의미: (현재 압력, 목표 토크) -> 다음 스텝 목표 압력.  레일은 리저버 동역학의 결과(공용, 상태).
tau_ach = L.Ppos.*sys.Apos.' - L.Pneg.*sys.Aneg.';   % Nt x N

varnames = {'time_s'};
data = {t};
for c = 1:sys.N
    varnames = [varnames, {sprintf('tau_target_N_ch%d',c), sprintf('Ppos_cur_kPa_ch%d',c), ...
        sprintf('Pneg_cur_kPa_ch%d',c), sprintf('Ppos_next_kPa_ch%d',c), ...
        sprintf('Pneg_next_kPa_ch%d',c), sprintf('tau_achieved_N_ch%d',c)}]; %#ok<AGROW>
    data = [data, {tau_traj(:,c), L.PposCur(:,c)/1e3, L.PnegCur(:,c)/1e3, ...
        L.Ppos(:,c)/1e3, L.Pneg(:,c)/1e3, tau_ach(:,c)}]; %#ok<AGROW>
end
varnames = [varnames, {'Ppos_rail_kPa','Pneg_rail_kPa','Ptank_kPa'}];
data = [data, {L.Ppp/1e3, L.Pnp/1e3, L.Ptk/1e3}];

T = table(data{:}, 'VariableNames', varnames);

disp('=== 결과 미리보기(처음 8행, 시간·공급원열만) ===');
disp(T(1:min(8,height(T)), {'time_s','Ppos_rail_kPa','Pneg_rail_kPa','Ptank_kPa'}));
writetable(T, 'reference_optimization_result.csv');
fprintf('CSV 저장: reference_optimization_result.csv (%d행, %d채널)\n', height(T), sys.N);

%% ---------------- 플롯: 채널별 토크 추종 (6채널 그리드) + 공급원 상태 ----------------
figure('Color','w','Position',[60 40 1300 850]);
for c = 1:sys.N
    subplot(3,2,c); hold on; grid on;
    plot(t, tau_traj(:,c), 'k','LineWidth',1.3);
    plot(t, tau_ach(:,c), 'g--','LineWidth',1.3);
    title(sprintf('축%d: target(시작 %.1fs, 크기 %.0fN, 폭 %.2fs)', c, specs(c,1), specs(c,2), specs(c,3)));
    ylabel('Force [N]'); if c>=5, xlabel('time [s]'); end
    if c==1, legend('target','achieved','Location','best'); end
end

figure('Color','w','Position',[60 40 1000 450]);
hold on; grid on;
plot(t, L.Ppp/1e3,'r','LineWidth',1.4);
plot(t, L.Pnp/1e3,'b','LineWidth',1.4);
plot(t, L.Ptk/1e3,'m','LineWidth',1.4);
plot(t, L.PppSP/1e3,'r--','LineWidth',0.9);      % 양압레일 셋포인트(적응)
plot(t, L.PnpSP/1e3,'b--','LineWidth',0.9);      % 음압레일 셋포인트(적응)
yline(sys.Pch_pos_max/1e3,'r:','액추에이터 양압정격');
yline(sys.Pch_neg_min/1e3,'b:','이젝터 음압한계');
ylabel('Pressure [kPa]'); xlabel('time [s]');
title('공급원 상태 & 적응 셋포인트: 레일은 프리뷰 수요비율로 능력경계 위 자동배분');
legend('P_{pos,rail}','P_{neg,rail}','P_{tank}','양압SP','음압SP','Location','best');
end


%% ======================================================================
%  [실시간 API #1] 1회 초기화: 파라미터 구성 + 펌프 능력 테이블 사전계산
%  실시간 시스템 기동 시 딱 한 번 호출하고, sys를 계속 재사용하세요.
%  ======================================================================
function sys = pressure_reference_init()
sys = build_params();
sys = build_pump_table(sys);
end


%% ======================================================================
%  [실시간 API #2] 매 제어 틱마다 1회 호출.
%    tau_target : 현재 목표 토크 (Nx1)
%    tau_preview: (선택) 미래 목표 토크 (N x H). 있으면 레일 셋포인트를 미리 배분.
%                 생략/빈배열이면 현재 목표만으로 결정(순수 실시간).
%  반환: 다음 스텝 채널 목표압력 + 갱신된 공급원 상태(양압레일/음압레일/탱크).
%  ======================================================================
function [ch_next, rail_next, rail_sp] = pressure_reference_step(tau_target, ch_prev, rail_prev, sys, tau_preview)
if nargin < 5 || isempty(tau_preview)
    tau_preview = tau_target;    % 프리뷰 없으면 현재 목표만
end
% 1) 레일 셋포인트: 프리뷰 수요의 양/음 비율로 펌프 능력경계 위 배분
rail_sp = decide_rail_setpoint(tau_preview, sys);
% 2) 채널 압력 최적화 (현재 상태만)
ch_next        = optimize_channels(tau_target, ch_prev, rail_prev, sys);
% 3) 공급원 상태 갱신 (릴리프는 rail_sp 기준으로 초과분만 방출)
[rail_next, ~] = update_sources(rail_prev, ch_prev, ch_next, rail_sp, sys);
end


%% ----------------------------------------------------------------------
%  레일 셋포인트 결정: 프리뷰(미래수요)의 '총 크기'로 양·음압 레일을 함께 조절.
%    이 시스템은 한 방향(+힘)만 내고, 음압도 +힘을 돕는 보조 -> '음압수요' 개념 없음.
%    - 총 수요 크면  -> 양쪽 레일 다 깊게(양압↑, 음압↓) 밀어 유량 여력 최대 확보
%    - 총 수요 작으면-> 과하게 안 밀고 여유만 (에너지/탱크 절약)
%    - 양압↑는 음압을 억누름(펌프 결합) -> 능력경계 위에서 함께 결정.
%  ----------------------------------------------------------------------
function rail_sp = decide_rail_setpoint(tau_preview, sys)
% tau_preview: N x H (H>=1), 목표는 항상 >=0 (한 방향 힘)
peak_each = max(tau_preview, [], 2);          % 각 액추 미래 최대 목표힘 (N x 1)
demand    = sum(peak_each);                    % 총 수요
demand_norm = min( demand / (sys.N * sys.Fmax_ref), 1.0 );   % 0~1

% 음압레일: 수요 클수록 깊게(여력↑), 작을수록 얕게(절약)
Pneg_shallow = -30e3;
pneg_sp = Pneg_shallow + demand_norm*(sys.Pneg_cap_deep - Pneg_shallow);
pneg_sp = max(sys.Pneg_cap_deep, min(Pneg_shallow, pneg_sp));

% 양압레일: 수요 클수록 높게(능력경계까지), 작을수록 여유만
ppos_target = sys.Ppos_sp_min + demand_norm*(sys.Ppos_sp_max - sys.Ppos_sp_min);
ppos_sp = min( cap_ppos(pneg_sp, sys), ppos_target );

rail_sp = [ppos_sp; pneg_sp];
end


%% ----------------------------------------------------------------------
%  펌프 능력경계 보간: 음압 셋포인트 -> 유지가능 최대 양압 (init에서 사전계산)
%  ----------------------------------------------------------------------
function pp = cap_ppos(pneg_sp, sys)
pp = interp1(sys.cap_pneg_grid, sys.cap_ppos_max, pneg_sp, 'linear', 'extrap');
pp = max(pp, 0);
end


%% ======================================================================
%  파라미터  (논문 Table A.1 + 업로드 MATLAB valve_params)
%  ======================================================================
function sys = build_params()
% ---- 채널/액추에이터 ----
sys.N    = 6;                          % 축 수 (6축)
sys.Apos = 1.0e-3*ones(sys.N,1);       % 양압 챔버 유효면적 [m^2] (다르면 축별로 값 교체)
sys.Aneg = 1.0e-3*ones(sys.N,1);       % 음압 챔버 유효면적 [m^2]
sys.Vpos = 0.75e-3*ones(sys.N,1);      % 양압 챔버 부피 [m^3] (논문 0.75 L)
sys.Vneg = 0.40e-3*ones(sys.N,1);      % 음압 챔버 부피 [m^3] (논문 0.4 L)

% ---- 액추에이터 정격(채널 목표압력 물리 상한) ----
sys.Pch_pos_max =  175e3;   % 양압 채널 최대 [Pa] (액추에이터 정격)
sys.Pch_neg_min =  -84e3;   % 음압 채널 최소 [Pa] (이젝터 도달한계 -84kPa)

% ---- 환경/공기 (논문 Table A.1) ----
sys.R     = 287;        % 이상기체상수 [J/(kg.K)]
sys.Tpis  = 323.15;     % 피스톤 온도 [K]
sys.Tch   = 293.15;     % 챔버 온도 [K]
sys.kappa = 1.4;        % 비열비
sys.Patm  = 101325;     % 대기압 [Pa]
sys.rho0  = 1.204;      % 표준 공기밀도 [kg/m^3] (LPM 환산용)

sys.dt = 1/50;          % 제어 주기 [s] (50Hz, 실시간 사용 시 실제 제어주기로 교체)
% ---- 폴리트로픽 지수 (열역학 가정) ----
%  [검토결과] 챔버는 1스텝(20ms) 동안 열전달 시상수(0.4~4s)보다 훨씬 빠르므로 '단열'이 맞다.
%  에너지식에서 엄밀히 유도:  dP/dt = (kappa*R/V)*(mdot_in*T_in - mdot_out*T_ch)
%  T_in≈T_ch 이면 등온 대비 정확히 kappa배. 반대로 레일/탱크는 수 초 규모로 변하므로 등온.
sys.n_ch   = 1.4;       % 챔버 폴리트로픽 지수 = kappa (단열)
sys.n_rail = 1.0;       % 레일/탱크 폴리트로픽 지수 (등온; 느린 동역학)
% 주의: 이는 온도를 상수로 두고 P만 추적하는 근사. 엄밀히는 챔버당 (P,T) 2상태 필요.

sys.Hpreview = 100;     % 프리뷰 창 [스텝] (100*20ms = 2s ≈ 레일 시상수)
%  [검토결과] H=25(0.5s)는 레일 시상수의 1/4뿐이라 효과를 절반도 못 봄.
%  H 스윕: 0.5s +9.1% / 1.0s +17.6% / 2.0s +20.4%(포화) -> 2s 채택.

% ---- 펌프 레일 셋포인트 배분 범위 (총 수요 크기로 능력경계 위에서 자동 결정) ----
% 이 시스템은 한 방향(+힘)만 냄. 음압도 +힘 보조 -> '음압수요' 개념 없음.
% 레일 목표는 고정이 아니라 매 스텝 프리뷰 '총 수요 크기'로 정함(decide_rail_setpoint):
%   총수요 클수록 양쪽 레일 다 깊게(양압↑,음압↓) 여력 확보 / 작으면 여유만.
sys.Pneg_cap_deep = -84e3;   % 음압 최대 깊이(이젝터 한계와 동일선) [Pa]
sys.Ppos_sp_min   = 100e3;   % 양압레일 최소 셋포인트(저수요 시) [Pa]
sys.Ppos_sp_max   = 400e3;   % 양압레일 최대 셋포인트(고수요 시) [Pa]
sys.Fmax_ref      = 150;     % 수요 정규화 기준: 액추 1개 '큰 힘'의 대표값 [N]
% (참고) 채널 정격은 Pch_pos_max=175k / Pch_neg_min=-84k 로 별도 제한됨.

% ---- 압축탱크(부스터) : 컴프레서 없음, 외부충전 후 소진 -> 동적 상태(처짐) ----
%  13ci(213mL)를 30MPa 충전 -> 정적 레귤레이터로 700kPa 출력(최대개방시에도 500k 유지).
%  탱크는 부스터 역할(레일이 처져 못 따라갈 때만 잠깐 사용) -> 사용량 최소화가 목표.
sys.V_tank      = 213e-6;   % 탱크 부피 [m^3] (13ci; 62ci옵션은 1016e-6)
sys.P_tank_sp   = 700e3;    % 레귤레이터 출력 셋포인트 [Pa]
sys.P_tank_min  = 500e3;    % 최대개방시에도 유지되는 하한(레귤레이터 특성) [Pa] (참고용)

% ---- 이젝터 (ZL112A) : 정압 싱크가 아니라 '진공도-유량 특성곡선' ----
%  [검토결과] -84kPa는 '무유량 최대 진공도'다. 이를 정압 싱크로 두고 4mm 오리피스를
%  붙이면 채널 0kPa에서 흡입 120 LPM을 요구해 정격(100 LPM)을 1.2배 초과하고,
%  더 심각하게는 실제로 유량이 흐를 때의 얕아진 진공도를 반영하지 못한다.
%  선형 특성곡선:  P_ej(Q) = P_ej_max * (1 - Q/Q_ej_max)
%  (예: 채널 0kPa일 때 실제 흡입 84 LPM에서 도달 진공은 -13.6kPa에 불과)
sys.P_ej_max  = -84e3;      % 무유량 최대 진공도 [Pa]
sys.Q_ej_max  = 100/60/1000;% 정격 흡입유량 [m^3/s] (100 LPM 표준상태)
sys.eject_ratio = 57/100;   % 소비/흡입 비 (탱크 소비 환산)
sys.use_ej_curve = true;    % false면 정압 싱크(구버전) 사용
%  ** 권장 **: 이젝터 후단 음압을 측정하고 있다면, 이 곡선 대신 측정값을 직접
%  하류 압력으로 넣는 것이 가장 정확하다 (sys.P_ej_meas 사용, 아래 참조).
sys.P_ej_meas = [];         % 실시간 측정 음압 [Pa]. 비어있지 않으면 이 값을 우선 사용.

% ---- (옵션) 피스톤 운동에 의한 체적변화 ----
%  [검토결과] 챔버를 고정체적으로 본 것은 근사다. 액추에이터가 움직이면
%  dP/dt = (nRT/V)*mdot - (n P/V)*dV/dt 의 둘째 항이 생긴다.
%  5kg/200mm 링크 조건 검증: 힘 감쇠(21.5ms)가 부하 운동보다 훨씬 빨라
%  힘 과도구간에서는 피스톤이 거의 안 움직이므로 고정체적 가정이 유효했다.
%  그러나 부하 관성이 작거나 스트로크가 짧으면 무시할 수 없다.
%  조인트 엔코더로 피스톤 속도를 알 수 있으면 아래에 넣어 정확도를 높일 수 있다.
sys.dVdt_pos = zeros(sys.N,1);   % 양압 챔버 체적변화율 [m^3/s] (신장>0)
sys.dVdt_neg = zeros(sys.N,1);   % 음압 챔버 체적변화율 [m^3/s]

% ---- 피스톤 펌프 (논문 식(1),(2),(3) + Table A.1) ----
sys.delta = 0.041;                 % 피스톤 최대 체적변화 길이 delta [m] (4.1 cm)
sys.r     = 0.02;                  % 크랭크 길이 r [m]
sys.l     = 0.07;                  % 링크 길이 l [m]
sys.omega = 3000*2*pi/60;          % 모터 각속도 [rad/s] (3000 rpm)
sys.Spis  = 38.485e-4;             % 피스톤 단면적 [m^2] (38.485 cm^2)
sys.Npis  = 2;                     % 피스톤 수 (180도 위상)
sys.Cbout = 1.46e-6;               % 토출 체크밸브 계수 Cb_out [m^2]
sys.Cbin  = 33.47e-6;              % 흡입 체크밸브 계수 Cb_in  [m^2]

% ---- 솔레노이드 밸브: 업로드 MATLAB 코드 valve_params 그대로 (목적함수 J2/J3 상대단위용) ----
% [A_max k_shape C_k C_p C_z A_bw beta_bw gamma_bw alpha_shape]
vp = [0.284504, 33.0944, 0.0288, 0.000124, 0.00000, 260649.5773, 179.0597, 0.0608, 3884.1953];
sys.A_max = vp(1);   % 최대 유효개도 계수 (완전개방 시 가용유량 기준, native/상대단위)

% ---- 펌프 능력 테이블 격자 (절대압 Pa) ----
% ---- 펌프 능력 테이블 격자 (절대압 Pa) - 동적 셋포인트 전 범위 커버 ----
sys.grid_pos = linspace(sys.Patm, sys.Patm + sys.Ppos_sp_max, 13);       % 양압 레일 abs
sys.grid_neg = linspace(sys.Patm + sys.Pneg_cap_deep, sys.Patm, 13);     % 음압 레일 abs
sys.pump_dt   = 1e-4;     % 펌프 적분 스텝 [s] (논문 ts)
sys.pump_nrev = 12;       % 정상 주기 도달용 회전수

% ---- 목적함수 가중치 ----
sys.wtrack  = 100;          % 토크 추종(소프트) 가중치 - 지배적으로 크게
sys.w_flow  = 0.3;         % J2: 총 유량 최대화(반응 여지 확보)
sys.w_fast  = 0.0;         % Jfast: '빠른 쪽 우선' -- 기본 0 (아래 검토결과 참조)
%  [검토결과] J_fast는 추종을 '해친다'. 목표가 도달 불가능(포화)한 순간에
%  최적 코너에서 해를 끌어내리기 때문. w_fast 스윕: 0 -> 0.188N, 0.5 -> 0.270N,
%  2 -> 0.466N, 10 -> 1.549N. 또 w_fast=0에서도 음압은 충분히 사용됨(힘의 33%,
%  최저 -73.5kPa). 슬루 박스가 이미 두 챔버의 물리 능력을 담고 있어서,
%  J_trk만으로도 '박스 안에서 최대한 목표에 접근'하면 자동으로 빠른 쪽을 쓴다.
%  동점(목표 도달 가능) 상황의 tie-break가 필요하면 아주 작은 값(<=0.05)만 사용.
sys.w_smooth= 0.5;         % J4: 압력 급변 억제(평활)
sys.w_tank  = 15;           % 탱크(부스트) 사용 페널티 - 레일 초과분에만 부과
sys.w_eject = 25;           % 이젝터 사용 페널티 - 탱크를 더 빨리 소모하므로 더 크게
%  [검토결과] '잔량 반비례(scarcity) 증폭'은 제거했다. 잔량이 줄수록 페널티를 키우면
%  성능이 서서히 저하되어 (a) 하위 MPC/RL에게 비정상(non-stationary) 플랜트가 되고
%  (b) 남은 자원을 쓰지도 못한다. 측정: 탱크 250k에서 scarcity ON 오차 3.997 vs
%  OFF 2.849 (33% 악화), 120k에서 4.201 vs 3.015 (39% 악화, 남은 120k 중 25k만 사용).
%  대신 '일정한 정책 + 명시적 하한 임계값'을 쓴다: 임계 이하로 떨어지면 교체/정지 신호.
sys.P_tank_stop = 450e3;   % 탱크 운전 하한 [Pa]. 이하면 usage.tank_low=true (교체/정지 판단)

% ---- 정규화 스케일 ----
sys.Pscale   = sys.Pch_pos_max;
sys.Tauscale = sys.Pch_pos_max * max(sys.Apos);                    % 토크 스케일(정격 기준)
sys.Qscale   = valve_capacity(sys.Ppos_sp_max+sys.Patm, sys.Patm, sys);
if sys.Qscale<=0, sys.Qscale=1; end

% ---- [아키텍처2] 공용 리저버(펌프 앞뒤 매니폴드) 부피 ----
sys.Vres_pos = 1.0e-3;      % 양압 리저버 부피 [m^3]
sys.Vres_neg = 1.0e-3;      % 음압 리저버 부피 [m^3]

% ---- 밸브별 물리 오리피스 (채널당 3+3, 릴리프 2 => 총 3N+2) ----
%  양압채널: (1)양압레일->채널 2.3mm  (2)채널->대기 4mm      (3)탱크->채널 1.6mm(부스트)
%  음압채널: (1)채널->음압레일 4mm    (2)대기->채널 4mm(완화) (3)이젝터<-채널 4mm(심화)
%  릴리프  : 양압레일->대기 1.6mm,  음압레일<-대기 1.6mm  (시스템 공용)
sys.Cd = 0.8;   % 방출계수(표준 가정치, 제조사 Cv 확보 시 교체 권장) - 전 경로 공통
Aorif = @(d) sys.Cd*pi*(d/2)^2;
sys.A_fill   = Aorif(2.3e-3);   % 양압레일 -> 채널
sys.A_vent   = Aorif(4.0e-3);   % 양압채널 -> 대기
sys.A_boost  = Aorif(1.6e-3);   % 탱크 -> 채널 (부스트)
sys.A_suck   = Aorif(4.0e-3);   % 채널 -> 음압레일
sys.A_admit  = Aorif(4.0e-3);   % 대기 -> 채널 (음압 완화)
sys.A_eject  = Aorif(4.0e-3);   % 이젝터 <- 채널 (음압 심화)
sys.A_bleed_pos = Aorif(1.6e-3);% 양압레일 -> 대기 (릴리프)
sys.A_bleed_neg = Aorif(1.6e-3);% 음압레일 <- 대기 (릴리프)
end


%% ======================================================================
%  한 스텝 최적화: 채널 압력만 결정 (레일은 리저버 동역학이 주는 상태)
%  ======================================================================
function ch_next = optimize_channels(tau_target, ch_prev, rail, sys)
% ch_prev = [Ppos(1..N); Pneg(1..N)],  rail = [Ppp; Pnp; Ptk]  (모두 게이지 Pa)
N=sys.N; nx=2*N;
Ppp=rail(1); Pnp=rail(2); Ptk=rail(3);
Pa=sys.Patm; rt=sys.n_ch*sys.R*sys.Tch;   % 챔버는 단열(n_ch=kappa)

lb=zeros(nx,1); ub=zeros(nx,1);
slewFpos=zeros(N,1); slewFneg=zeros(N,1);   % 이번스텝 힘 증가능력(빠름 척도)
for i=1:N
    Pp0=ch_prev(i); Pn0=ch_prev(N+i);
    % 양압 상승: 레일(2.3mm) + 탱크부스트(1.6mm) 동시 사용 가능 -> 합산
    mfill = valve_phys_kgps(Ppp+Pa, Pp0+Pa, sys.A_fill,  sys);   % 레일->채널
    mboost= valve_phys_kgps(Ptk+Pa, Pp0+Pa, sys.A_boost, sys);   % 탱크->채널
    mvent = valve_phys_kgps(Pp0+Pa, Pa,     sys.A_vent,  sys);   % 채널->대기
    % 음압 심화: 음압레일(4mm) + 이젝터(특성곡선) 동시 사용 가능 -> 합산
    msuck = valve_phys_kgps(Pn0+Pa, Pnp+Pa, sys.A_suck,  sys);   % 채널->음압레일
    meject= ejector_flow(Pn0, sys);                              % 채널->이젝터
    madmit= valve_phys_kgps(Pa,     Pn0+Pa, sys.A_admit, sys);   % 대기->채널(완화)

    % 피스톤 운동에 의한 체적변화 항: dP += -(n*P/V)*dV/dt
    volP = sys.n_ch*(Pp0+Pa)/sys.Vpos(i)*sys.dVdt_pos(i)*sys.dt;
    volN = sys.n_ch*(Pn0+Pa)/sys.Vneg(i)*sys.dVdt_neg(i)*sys.dt;

    dP_up=(mfill+mboost)*sys.dt*rt/sys.Vpos(i) - volP;
    dP_dn= mvent        *sys.dt*rt/sys.Vpos(i) + volP;
    dN_dn=(msuck+meject)*sys.dt*rt/sys.Vneg(i) + volN;
    dN_up= madmit       *sys.dt*rt/sys.Vneg(i) - volN;
    dP_up=max(dP_up,0); dP_dn=max(dP_dn,0); dN_dn=max(dN_dn,0); dN_up=max(dN_up,0);

    % 상/하한: 채널은 액추에이터 정격(175 / -84) 안에서만
    loP=max(0,               Pp0-dP_dn);  hiP=min(sys.Pch_pos_max, Pp0+dP_up);  if hiP<loP, hiP=loP; end
    loN=max(sys.Pch_neg_min, Pn0-dN_dn);  hiN=min(0,               Pn0+dN_up);  if hiN<loN, hiN=loN; end
    lb(i)=loP; ub(i)=hiP; lb(N+i)=loN; ub(N+i)=hiN;

    % 빠름 척도: 이번 스텝에 낼 수 있는 힘 증가량 (상승방향, 클수록 빠른 채널)
    slewFpos(i)=max(dP_up*sys.Apos(i), 1e-6);
    slewFneg(i)=max(dN_dn*sys.Aneg(i), 1e-6);
end

xs=1e5;   % 스케일: 압력 O(1e5)->O(1) (fmincon 그래디언트 정상화)
fS = @(z) obj_fun_ch(z*xs, ch_prev, tau_target, rail, slewFpos, slewFneg, sys);
x0 = min(max(ch_prev, lb), ub);

opts = optimoptions('fmincon','Algorithm','sqp','Display','off', ...
        'MaxIterations',200,'MaxFunctionEvaluations',3000, ...
        'OptimalityTolerance',1e-10,'StepTolerance',1e-12);
[zstar,~,exitflag] = fmincon(fS, x0/xs, [],[], [],[], lb/xs, ub/xs, [], opts);
ch_next = zstar*xs;
if exitflag <= 0
    warning('fmincon exitflag=%d (수렴 실패 가능)', exitflag);
end
end


%% ----------------------------------------------------------------------
%  공급원 상태 갱신: 양압레일, 음압레일, 압축탱크(부스터)
%    - 채널 양압상승 = 레일이 최대한 + 초과분은 탱크(부스트)
%    - 채널 음압심화 = 음압레일이 최대한 + 초과분은 이젝터
%    - 탱크는 부스트유량 + 이젝터소비(결합)만큼 감소, 컴프레서 없어 회복 없음
%    - 릴리프(1.6mm)는 셋포인트 근방에서 초과분만 방출
%  ----------------------------------------------------------------------
function [rail_next, usage] = update_sources(rail, ch_prev, ch_next, rail_sp, sys)
N=sys.N; Pa=sys.Patm;
rt   = sys.n_ch  *sys.R*sys.Tch;   % 챔버 수요 환산(단열)
rt_r = sys.n_rail*sys.R*sys.Tch;   % 레일/탱크 적분(등온)
Ppp=rail(1); Pnp=rail(2); Ptk=rail(3);
Ppos_sp=rail_sp(1); Pneg_sp=rail_sp(2);   % 이 스텝의 동적 레일 셋포인트

mfill_dem=0; mboost_dem=0; msuck_dem=0; meject_dem=0;
for i=1:N
    % 양압 상승분을 레일이 최대한, 초과분은 탱크
    dup = max(0, ch_next(i)-ch_prev(i)) * sys.Vpos(i)/rt / sys.dt;   % 필요 질량유량
    mfill_cap = valve_phys_kgps(Ppp+Pa, ch_prev(i)+Pa, sys.A_fill, sys);
    mfill  = min(dup, mfill_cap);  mboost = max(0, dup - mfill);
    mfill_dem  = mfill_dem  + mfill;   mboost_dem = mboost_dem + mboost;
    % 음압 심화분을 음압레일이 최대한, 초과분은 이젝터 (이젝터는 특성곡선)
    ddn = max(0, ch_prev(N+i)-ch_next(N+i)) * sys.Vneg(i)/rt / sys.dt;
    msuck_cap = valve_phys_kgps(ch_prev(N+i)+Pa, Pnp+Pa, sys.A_suck, sys);
    mej_cap   = ejector_flow(ch_prev(N+i), sys);
    msuck  = min(ddn, msuck_cap);
    meject = min(max(0, ddn - msuck), mej_cap);   % 이젝터도 정격 이내로 제한
    msuck_dem  = msuck_dem  + msuck;   meject_dem = meject_dem + meject;
end

mp_pos = max(0, sys.Fpos(Ppp+Pa, Pnp+Pa));   % 펌프 양압 공급 [kg/s]
mp_neg = max(0, sys.Fneg(Ppp+Pa, Pnp+Pa));   % 펌프 음압 흡입 [kg/s]

% 양압레일 릴리프: 동적 셋포인트(Ppos_sp) 초과분만 방출
excess_pos = mp_pos - mfill_dem; bleed_pos = 0;
if Ppp >= Ppos_sp - 1e3 && excess_pos > 0
    bleed_pos = min(excess_pos, valve_phys_kgps(Ppp+Pa, Pa, sys.A_bleed_pos, sys));
end
% 음압레일 릴리프: 동적 셋포인트(Pneg_sp)보다 깊어지면 대기유입으로 완화
excess_neg = mp_neg - msuck_dem; bleed_neg = 0;
if Pnp <= Pneg_sp + 1e3 && excess_neg > 0
    bleed_neg = min(excess_neg, valve_phys_kgps(Pa, Pnp+Pa, sys.A_bleed_neg, sys));
end

% 레일/탱크는 느린 동역학 -> 등온(rt_r)
Ppp_next = max(Ppp + sys.dt*rt_r/sys.Vres_pos*(mp_pos - mfill_dem - bleed_pos), 0);
Pnp_next = min(Pnp + sys.dt*rt_r/sys.Vres_neg*(-(mp_neg - msuck_dem) + bleed_neg), 0);

% 탱크: 부스트 + 이젝터소비(결합, 소비/흡입=57/100 환산)만큼 감소. 회복 없음.
m_eject_tank = meject_dem * sys.eject_ratio;
Ptk_next = Ptk - sys.dt*rt_r/sys.V_tank*(mboost_dem + m_eject_tank);
Ptk_next = min(max(Ptk_next, 0), sys.P_tank_sp);   % 셋포인트 이상은 레귤레이터가 막음

rail_next = [Ppp_next; Pnp_next; Ptk_next];
usage = struct('fill',mfill_dem,'boost',mboost_dem,'suck',msuck_dem, ...
               'eject',meject_dem,'tank_draw',mboost_dem+m_eject_tank, ...
               'tank_low', Ptk_next < sys.P_tank_stop);   % 교체/정지 판단 플래그
end


%% ----------------------------------------------------------------------
%  목적함수 (J0 토크추종 + J2,J3,J4) - 채널 변수만, 레일은 주어짐
%  ----------------------------------------------------------------------
function J = obj_fun_ch(ch, ch_prev, tau_target, rail, slewFpos, slewFneg, sys)
N=sys.N; Ppos=ch(1:N); Pneg=ch(N+1:2*N);
Ppp=rail(1); Pnp=rail(2); Ptk=rail(3); Pa=sys.Patm; rt=sys.n_ch*sys.R*sys.Tch;

tau_ach = Ppos.*sys.Apos - Pneg.*sys.Aneg;
J0 = sum( ((tau_ach - tau_target)/sys.Tauscale).^2 );

% J2) 총 가용유량 최대화(반응 여지)
Qpos=zeros(N,1); Qneg=zeros(N,1);
for i=1:N
    Qpos(i) = valve_capacity(Ppp+Pa,     Ppos(i)+Pa, sys);
    Qneg(i) = valve_capacity(Pneg(i)+Pa, Pnp+Pa,      sys);
end
J2 = -(sum(Qpos)+sum(Qneg)) / sys.Qscale;

% Jfast) '빠른 쪽 우선' 분해: 힘을 낼 때 느린 채널(슬루 작음)을 쓰면 페널티.
%   비용 = (그 채널로 낸 힘 증가분) / (그 채널의 이번스텝 힘 증가능력).
%   -> 빠른 채널로 낼수록 비용 작음 => 최적화가 빠른 채널로 토크를 몰아줌.
Jfast = 0;
for i=1:N
    dfpos = max(0, (Ppos(i)   - ch_prev(i))  *sys.Apos(i));  % 양압으로 낸 힘증가
    dfneg = max(0, (ch_prev(N+i) - Pneg(i))  *sys.Aneg(i));  % 음압으로 낸 힘증가
    Jfast = Jfast + dfpos/slewFpos(i) + dfneg/slewFneg(i);
end
Jfast = Jfast / N;

J4 = sum(abs(ch - ch_prev)) / sys.Pscale;

% ---- 탱크/이젝터 절약: "레일이 못 대는 초과분"에만 페널티 ----
%  scarcity(잔량 반비례) 증폭은 제거. 정책을 일정하게 유지하고, 잔량 하한은
%  update_sources의 tank_low 플래그로 별도 처리 (교체/정지 판단).
Jtank=0; Jeject=0;
for i=1:N
    dPp = max(0, Ppos(i)   - ch_prev(i));
    dPn = max(0, ch_prev(N+i) - Pneg(i));
    mfill_cap = valve_phys_kgps(Ppp+Pa, ch_prev(i)+Pa,   sys.A_fill, sys);
    dP_rail_max = mfill_cap*sys.dt*rt/sys.Vpos(i);
    Jtank  = Jtank  + max(0, dPp - dP_rail_max)/sys.Pch_pos_max;
    msuck_cap = valve_phys_kgps(ch_prev(N+i)+Pa, Pnp+Pa, sys.A_suck, sys);
    dN_rail_max = msuck_cap*sys.dt*rt/sys.Vneg(i);
    Jeject = Jeject + max(0, dPn - dN_rail_max)/abs(sys.Pch_neg_min);
end

J = sys.wtrack*J0 + sys.w_flow*J2 + sys.w_fast*Jfast + sys.w_smooth*J4 ...
    + sys.w_tank*Jtank + sys.w_eject*Jeject;
end


%% ----------------------------------------------------------------------
%  채널별 밸브 "가용유량" (플롯용, 목적함수 상대단위) - 업로드 MATLAB 오리피스식 포팅
%  ----------------------------------------------------------------------
function [Qpos,Qneg] = channel_flows(ch, rail, sys)
N=sys.N; Pa=sys.Patm; Ppp=rail(1); Pnp=rail(2);
Qpos=zeros(N,1); Qneg=zeros(N,1);
for i=1:N
    Qpos(i) = valve_capacity(Ppp+Pa,        ch(i)+Pa,   sys);
    Qneg(i) = valve_capacity(ch(N+i)+Pa,    Pnp+Pa,     sys);
end
end


%% ======================================================================
%  [이젝터 유량]  진공도-유량 특성곡선과 오리피스 유량의 교점을 구한다.
%    P_ej(Q) = P_ej_max*(1 - Q/Q_ej_max)  <-> mdot = A*P_up/sqrt(RT)*Phi(P_ej/P_up)
%  측정값(sys.P_ej_meas)이 있으면 그것을 하류 압력으로 그대로 사용(가장 정확).
%  ======================================================================
function [m, P_ej] = ejector_flow(Pch_gauge, sys)
Pa=sys.Patm;
if ~isempty(sys.P_ej_meas)                 % 측정값 우선
    P_ej = sys.P_ej_meas;
    m = valve_phys_kgps(Pch_gauge+Pa, P_ej+Pa, sys.A_eject, sys);
    return
end
if ~sys.use_ej_curve                        % 구버전: 정압 싱크
    P_ej = sys.P_ej_max;
    m = valve_phys_kgps(Pch_gauge+Pa, P_ej+Pa, sys.A_eject, sys);
    return
end
% 특성곡선과의 교점 (이분법). Q는 표준상태 체적유량으로 환산해 비교.
lo=0; hi=sys.Q_ej_max;
for it=1:40
    Q=0.5*(lo+hi);
    P_ej = sys.P_ej_max*(1 - Q/sys.Q_ej_max);
    m    = valve_phys_kgps(Pch_gauge+Pa, P_ej+Pa, sys.A_eject, sys);
    Qo   = m/sys.rho0;                      % kg/s -> 표준 m^3/s
    if Qo > Q, lo=Q; else, hi=Q; end
end
Q=0.5*(lo+hi);
P_ej = sys.P_ej_max*(1 - Q/sys.Q_ej_max);
m    = valve_phys_kgps(Pch_gauge+Pa, P_ej+Pa, sys.A_eject, sys);
end


%% ======================================================================
%  [밸브 물리 유량, kg/s]  펌프 체크밸브(식3)와 동일 형태, 물리 오리피스 면적 사용
%  -> 슬루 한도·리저버 수지에 사용 (목적함수 J2/J3의 native valve_capacity와는 별개)
%  ======================================================================
function m = valve_phys_kgps(Pup_abs, Pdn_abs, A, sys)
m = A * Pup_abs / sqrt(sys.R*sys.Tch) * orifice_phi(Pup_abs, Pdn_abs, sys.kappa);
end


%% ======================================================================
%  [밸브 유량식 포팅]  Q = A_max * P_up * Phi(Pr)   (valve_step_v2 와 동일)
%  완전개방(Area_eff=A_max) 기준 가용유량. P_up,P_dn = 절대압[Pa].
%  ======================================================================
function Q = valve_capacity(Pup_abs, Pdn_abs, sys)
phi = orifice_phi(Pup_abs, Pdn_abs, sys.kappa);
Q   = sys.A_max * Pup_abs * phi;          % native 단위 (J2/J3는 정규화하여 사용)
end


%% ======================================================================
%  [펌프 유량식 포팅]  논문 식 (1),(2),(3)
%  주어진 레일 절대압에서 피스톤 1주기 평균 공급/흡입 질량유량 [kg/s]
%  ======================================================================
function [mdot_out_avg, mdot_in_avg] = pump_piston_avg(Ppos_abs, Pneg_abs, sys)
R=sys.R; Tp=sys.Tpis; ka=sys.kappa; w=sys.omega;
Sp=sys.Spis; del=sys.delta; r=sys.r; l=sys.l;
Cbout=sys.Cbout; Cbin=sys.Cbin;
dt=sys.pump_dt; Trev=2*pi/w; nstep=round(sys.pump_nrev*Trev/dt);

Vp = @(th) Sp.*(del - r + l - r.*cos(th) - sqrt(l^2 - r^2*sin(th).^2));  % 식(2) V_pis

th = 0;
m  = sys.Patm * Vp(th) / (R*Tp);   % 초기: 대기압 상당 질량
acc_out = 0; acc_in = 0; t_acc = 0;
t_lastrev = (sys.pump_nrev-1)*Trev;

for s = 1:nstep
    V    = Vp(th);
    Ppis = m*R*Tp / V;                                  % 식(1) P_pis

    % 토출 체크밸브: 피스톤 -> 양압레일 (Ppis > Ppos)
    if Ppis > Ppos_abs
        mdot_out = Cbout * Ppis / sqrt(R*Tp) * orifice_phi(Ppis, Ppos_abs, ka); % 식(3)
    else
        mdot_out = 0;
    end
    % 흡입 체크밸브: 음압레일 -> 피스톤 (Pneg > Ppis)
    if Pneg_abs > Ppis
        mdot_in = Cbin * Pneg_abs / sqrt(R*Tp) * orifice_phi(Pneg_abs, Ppis, ka); % 식(3)
    else
        mdot_in = 0;
    end

    m  = m + (mdot_in - mdot_out)*dt;                   % dm/dt = m_in - m_out
    th = th + w*dt;

    if (s*dt) > t_lastrev                               % 마지막 1주기 평균
        acc_out = acc_out + mdot_out*dt;
        acc_in  = acc_in  + mdot_in *dt;
        t_acc   = t_acc + dt;
    end
end
if t_acc<=0, t_acc=Trev; end
mdot_out_avg = sys.Npis * acc_out / t_acc;
mdot_in_avg  = sys.Npis * acc_in  / t_acc;
end


%% ----------------------------------------------------------------------
%  펌프 공급능력 2D 테이블 사전계산 -> griddedInterpolant
%  ----------------------------------------------------------------------
function sys = build_pump_table(sys)
gp=sys.grid_pos; gn=sys.grid_neg;
Mout=zeros(numel(gp),numel(gn));
Min =zeros(numel(gp),numel(gn));
fprintf('펌프 공급능력 테이블 계산 중 (%dx%d)...\n', numel(gp),numel(gn));
for a=1:numel(gp)
    for b=1:numel(gn)
        [Mout(a,b), Min(a,b)] = pump_piston_avg(gp(a), gn(b), sys);
    end
end
[GP,GN]=ndgrid(gp,gn);
sys.Fpos = griddedInterpolant(GP,GN,Mout,'linear','nearest');  % 양압 공급 [kg/s]
sys.Fneg = griddedInterpolant(GP,GN,Min ,'linear','nearest');  % 음압 흡입 [kg/s]
fprintf('완료. 최대 공급 %.4g g/s\n', 1e3*max(Mout(:)));

% ---- 능력경계 테이블: 각 음압 셋포인트에서 펌프가 닿는 최대 양압 ----
%  (레일 셋포인트 자동배분 decide_rail_setpoint 에서 사용)
sys.cap_pneg_grid = linspace(sys.Pneg_cap_deep, -30e3, 21);
sys.cap_ppos_max  = zeros(size(sys.cap_pneg_grid));
for j=1:numel(sys.cap_pneg_grid)
    pn = sys.cap_pneg_grid(j); pmax=0;
    for pp = 0:5e3:sys.Ppos_sp_max
        mo = pump_piston_avg(pp+sys.Patm, pn+sys.Patm, sys);
        if mo*1e3 > 0.02, pmax=pp; end
    end
    sys.cap_ppos_max(j) = pmax;
end
end


%% ======================================================================
%  [공통] 압축성 오리피스 Phi  (논문 식(3) = 업로드 MATLAB 코드와 동일)
%  ======================================================================
function phi = orifice_phi(Pin, Pout, kappa)
if Pin <= 0 || Pout <= 0, phi=0; return; end
Pr = Pout/Pin;
if Pr > 1, phi=0; return; end                 % 역류 차단
Pcr = (2/(kappa+1))^(kappa/(kappa-1));
if Pr <= Pcr
    phi = sqrt(kappa*(2/(kappa+1))^((kappa+1)/(kappa-1)));   % choked
else
    phi = sqrt(2*kappa/(kappa-1)) * sqrt(Pr^(2/kappa) - Pr^((kappa+1)/kappa)); % subsonic
end
end


%% ----------------------- (끝) ------------------------------------------
