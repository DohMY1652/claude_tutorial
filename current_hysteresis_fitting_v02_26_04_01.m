% =========================================================================
% 비례 밸브 물리 모델 기반 전역 최적화 (정적 9변수 + 동적 4변수 = 총 13변수)
% - 절대압 데이터 덧셈 버그 수정 및 초기 유량 동기화 적용
% - 마이크로 스텝(Sub-stepping) 적용으로 최적화 중 발산 방지
% =========================================================================
clear; clc; close all;

%% 1. 데이터 로드 및 전처리
% =========================================================================
% [데이터 자르기 설정] 
% 🌟 주의: 가급적 밸브가 완전히 닫혀 유량이 0인 시점을 시작 시간으로 설정하세요!
start_time_sec = 200.0; 
% =========================================================================
[files, folder] = uigetfile('*.csv', '검증용 데이터 CSV 파일들을 모두 선택하세요', 'MultiSelect', 'on');
if isequal(files, 0), return; end
if ischar(files), files = {files}; end
num_files = length(files);
all_data = cell(num_files, 1);

fprintf('%d개의 데이터 중 %.1f초 이후의 구간만 잘라서 최적화를 시작합니다...\n', num_files, start_time_sec);

I_MAX = 0.30; % 정규화 입력(u=1.0)에 매핑될 최대 전류 (0.3A)
window_size = 20; 

for i = 1:num_files
    filename = fullfile(folder, files{i});
    raw = readmatrix(filename);
    
    temp.name = files{i};
    
    % 1. 전체 시간 확인 및 인덱스 자르기
    all_time = raw(:, 1);
    valid_idx = all_time >= start_time_sec;
    
    if sum(valid_idx) == 0
        warning('%s 파일은 %.1f초 이후의 데이터가 없습니다. 전체 데이터를 사용합니다.', files{i}, start_time_sec);
        valid_idx = true(size(all_time));
    end
    
    % 2. CSV 열 파싱 (잘라낸 인덱스 구간만 추출)
    temp.Time = raw(valid_idx, 1); 
    temp.u = max(min((raw(valid_idx, 2)-0.5)*2.0, 1.0),0.0);
    
    % 🌟 [수정됨] 데이터가 이미 절대압이므로 P_atm(101.325)을 더하지 않고 바로 저장합니다.
    temp.P_in_abs = raw(valid_idx, 3);
    temp.P_out_abs = raw(valid_idx, 4);
    temp.Q = raw(valid_idx, 5); 
    
    temp.dt = mean(diff(temp.Time));
    
    % 정규화 입력(u) -> 실제 전류(I) 변환 및 노이즈 필터링
    temp.u(temp.u < 0) = 0; temp.u(temp.u > 1) = 1;
    temp.I = temp.u * I_MAX;
    temp.I = movmean(temp.I, window_size);
    temp.Q = movmean(temp.Q, window_size);
    
    % 압축성 유동 Phi 사전 계산
    temp.Phi = get_phi(temp.P_in_abs, temp.P_out_abs, 1.4);
    
    % 방향(State) 판별
    N = length(temp.I);
    temp.State = zeros(N, 1);
    for k = 2:N
        if temp.I(k) > temp.I(k-1) + 1e-4
            temp.State(k) = 1; % 열림
        elseif temp.I(k) < temp.I(k-1) - 1e-4
            temp.State(k) = 0; % 닫힘
        else
            temp.State(k) = temp.State(k-1);
        end
    end
    temp.State(1) = temp.State(2);
    
    all_data{i} = temp;
end

%% 2. 최적화 설정
% 파라미터 순서 (13개):
% [A_max, k_shape, C_k, C_p, C_z, A_bw, beta_bw, gamma_bw, alpha_shape, wn_up, zeta_up, wn_down, zeta_down]
base_initial = [0.2845, 33.09, 0.0288, 0.00012, 0.0, 260649.5, 179.0, 0.06, 3884.2, 40.0, 1.2, 45.0, 1.0];
options = optimset('MaxIter', 15000, 'MaxFunEvals', 50000, 'TolFun', 1e-6, 'TolX', 1e-6, 'Display', 'off');
global_cost_func = @(params) compute_global_error(params, all_data);

%% 3. [1단계] 전역 난수 탐색 (Global Random Search)
num_samples = 200; 
sample_results = zeros(num_samples, length(base_initial) + 1);
disp('1단계: 전역 난수 탐색 중... (가장 유망한 시작점 3곳 선정)');
for s = 1:num_samples
    if s == 1
        guess = base_initial;
    else
        noise = 0.5 + (1.5 - 0.5) * rand(1, length(base_initial));
        guess = base_initial .* noise;
        guess(5) = (rand - 0.5) * 0.1;   % C_z 부호 랜덤화
        guess(10) = 10 + rand * 50;      % wn_up 랜덤화
        guess(12) = 10 + rand * 50;      % wn_down 랜덤화
    end
    err = global_cost_func(guess);
    sample_results(s, :) = [guess, err];
end
sample_results = sortrows(sample_results, length(base_initial) + 1);
top_starts = sample_results(1:3, 1:length(base_initial));

%% 4. [2단계] 정밀 피팅 (Local Fine-Tuning)
disp('2단계: 상위 시작점 기반 초정밀 최적화 수행 중... (시간이 다소 소요됩니다)');
best_error = inf; best_params = base_initial;
for i = 1:size(top_starts, 1)
    [current_params, current_error] = fminsearch(global_cost_func, top_starts(i,:), options);
    if current_error < best_error
        best_error = current_error;
        best_params = current_params;
    end
    fprintf('  - 정밀 탐색 %d/3 완료 (현재 최소 오차 SS_res: %.2f)\n', i, best_error);
end

%% 5. 최종 파라미터 추출 및 결과 출력
opt_p = best_params;
disp('====================================================');
disp('      [디지털 트윈 13변수 물리 모델 최적화 완료]    ');
disp('====================================================');
fprintf('A_max      = %.6f\n', abs(opt_p(1)));
fprintf('k_shape    = %.4f\n', abs(opt_p(2)));
fprintf('C_k        = %.4f\n', abs(opt_p(3)));
fprintf('C_p        = %.6f\n', opt_p(4));
fprintf('C_z        = %.5f\n', opt_p(5));
fprintf('A_bw       = %.4f\n', abs(opt_p(6)));
fprintf('beta_bw    = %.4f\n', abs(opt_p(7)));
fprintf('gamma_bw   = %.4f\n', abs(opt_p(8)));
fprintf('alpha_shape= %.4f\n', abs(opt_p(9)));
fprintf('wn_up      = %.4f, zeta_up   = %.4f\n', min(abs(opt_p(10)), 150), abs(opt_p(11)));
fprintf('wn_down    = %.4f, zeta_down = %.4f\n', min(abs(opt_p(12)), 150), abs(opt_p(13)));

%% 6. R-square 계산 및 최종 시각화
cols = ceil(sqrt(num_files)); rows = ceil(num_files / cols);
figure('Color', 'w', 'Position', [100, 100, 1400, 900], 'Name', 'Digital Twin Optimization Fit');
global_SS_res = 0; global_SS_tot = 0; all_Q_actual = []; 
for i = 1:num_files, all_Q_actual = [all_Q_actual; all_data{i}.Q]; end
global_Q_mean = mean(all_Q_actual);

for i = 1:num_files
    data = all_data{i};
    [~, Q_pred] = simulate_physics_model(data, best_params);
    
    SS_res_local = sum((data.Q - Q_pred).^2);
    SS_tot_local = sum((data.Q - mean(data.Q)).^2);
    R_sq_local = 1 - (SS_res_local / SS_tot_local);
    RMSE_local = sqrt(mean((data.Q - Q_pred).^2));
    
    global_SS_res = global_SS_res + SS_res_local;
    global_SS_tot = global_SS_tot + sum((data.Q - global_Q_mean).^2);
    
    subplot(rows, cols, i);
    plot(data.Time, data.Q, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Actual'); hold on;
    plot(data.Time, Q_pred, 'r--', 'LineWidth', 2.0, 'DisplayName', 'Fitted');
    
    grid on; xlabel('Time (s)'); ylabel('Flow (LPM)');
    title(sprintf('%s\n(R^2 = %.2f%%, RMSE = %.2f)', data.name, R_sq_local*100, RMSE_local), 'Interpreter', 'none', 'FontSize', 10);
    legend('Location', 'best');
end
Global_R_sq = 1 - (global_SS_res / global_SS_tot);
disp('====================================================');
fprintf('★ 통합 글로벌 모델 정확도 (Global R-square) : %.4f (%.2f%%)\n', Global_R_sq, Global_R_sq * 100);
disp('====================================================');

% =========================================================================
% [로컬 함수] 글로벌 오차 연산 (Cost Function)
% =========================================================================
function total_error = compute_global_error(params, all_data)
    total_error = 0;
    for i = 1:length(all_data)
        [error, ~] = simulate_physics_model(all_data{i}, params);
        total_error = total_error + error;
    end
end

% =========================================================================
% [로컬 함수] 13변수 물리 모델 단일 시뮬레이션
% =========================================================================
function [error, Q_pred] = simulate_physics_model(data, params)
    A_max   = abs(params(1)); k_shape = abs(params(2)); C_k = abs(params(3));
    C_p     = params(4);      C_z     = params(5);      
    A_bw    = abs(params(6)); beta_bw = abs(params(7)); gamma_bw = abs(params(8)); 
    alpha   = abs(params(9)); 
    
    wn_up   = min(abs(params(10)), 150); zeta_up   = abs(params(11));
    wn_down = min(abs(params(12)), 150); zeta_down = abs(params(13));
    
    N = length(data.I); 
    Q_pred = zeros(N, 1); 
    z = 0; 
    
    % 🌟 [수정됨] 초기 유량(x1)을 잘라낸 데이터의 첫 번째 실제 유량으로 강제 동기화
    x1 = data.Q(1); 
    x2 = 0; 
    
    % 🌟 [수정됨] 최적화 속도를 고려하여 20번의 마이크로 스텝 적용 (발산 방지)
    num_sub_steps = 20; 
    dt_sub = data.dt / num_sub_steps;
    
    for k = 1:N
        if k > 1
            abs_dI = abs(data.I(k) - data.I(k-1));
            dI = abs_dI * (2*data.State(k) - 1); 
            dz = A_bw * dI - beta_bw * abs(dI) * z - gamma_bw * dI * abs(z);
            z = z + dz;
        end
        if z > 1e6, z = 1e6; elseif z < -1e6, z = -1e6; end
        
        Force_net = data.I(k) + C_z * z + C_p * data.P_in_abs(k) - C_k; % 게이지압 -> 절대압 변수 사용으로 수정
        if Force_net < -500, Force_net = -500; elseif Force_net > 500, Force_net = 500; end
        
        Area_eff = A_max / ((1 + exp(-k_shape * Force_net))^alpha);
        Q_static = Area_eff * data.P_in_abs(k) * data.Phi(k);
        
        if data.State(k) == 1
            wn = wn_up; zeta = zeta_up;
        else
            wn = wn_down; zeta = zeta_down;
        end
        
        % 오일러 적분 마이크로 스텝 진행
        for sub = 1:num_sub_steps
            dx1 = x2;
            dx2 = wn^2 * (Q_static - x1) - 2 * zeta * wn * x2;
            
            x1 = x1 + dt_sub * dx1;
            x2 = x2 + dt_sub * dx2;
        end
        
        Q_pred(k) = x1;
    end
    
    error = sum((data.Q - Q_pred).^2);  
    if Q_pred(2) > mean(data.Q) * 2, error = error + 1e7; end
end

% =========================================================================
% [로컬 함수] 압축성 유동 (Phi) 연산
% =========================================================================
function phi = get_phi(P_in, P_out, kappa)
    Pr = P_out ./ P_in;
    Pr(Pr > 1.0) = 1.0; 
    P_cr = (2 / (kappa + 1))^(kappa / (kappa - 1));
    phi = zeros(size(Pr));
    
    idx_choked = (Pr <= P_cr);
    phi(idx_choked) = sqrt(kappa * (2 / (kappa + 1))^((kappa + 1) / (kappa - 1)));
    
    idx_sub = (Pr > P_cr) & (Pr <= 1);
    term1 = sqrt((2 * kappa) / (kappa - 1));
    term2 = sqrt(Pr(idx_sub).^(2 / kappa) - Pr(idx_sub).^((kappa + 1) / kappa));
    phi(idx_sub) = term1 .* term2;
end