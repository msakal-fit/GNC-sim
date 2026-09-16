% filename: MATLAB/GNC_Algorithms/Control/LQG/test_p09_lqg_offline.m

% test script for offline test of LQG controller for quadcopter hover

clear; clc; close all;

% load quadcopter parameters
px4_config = get_x500_params();

% timestep setting
Ts = 0.01;  % [s] update and simulation timestep
T_sim = 20.0;  % [s] total simulation time
N_sim = round(T_sim / Ts);
t_hist = (0:N_sim) * Ts;

% LQR weights
Q_pos = diag([ 4,  4,  8]);          % px, py, pz  (pz more important)
Q_vel = diag([ 1,  1,  2]);          % vx, vy, vz  (vz a bit more)
Q_q   = diag([ 40, 20, 20, 20]);     % q0, q1, q2, q3
Q_om  = diag([ 2,   2,   2]);        % p, q, r

Q_lqr = blkdiag(Q_pos, Q_vel, Q_q, Q_om);

R_lqr = diag([0.5, 15, 15, 15]);     % [T, τx, τy, τz]

% equilibrium input (thrust during hover)
m = px4_config.m;
g = px4_config.g;
U_eq = [m * g; 0; 0; 0];

% state reference (hover at -5m)
x_ref = zeros(13,1);
x_ref(3) = -5.0;     % desired altitude
% level attitude: q0 = 1, [q1 q2 q3] = 0
x_ref(7) = 1.0;     % q0
x_ref(8:10) = 0.0;  % q1, q2, q3
% velocities and angular rates are set to zero

% inital true state
x_true = zeros(13,1);
x_true(1:3) = [5.0; -0.5; -2.0];   % initial position error
q_init = [1.0; 0; 0; 0.0];         % small attitude error
q_init = q_init / norm(q_init);
x_true(7:10) = q_init;


% Kalman filter initial conditions
x_hat = zeros(13,1);  % initial state estimate
P = 0.1 * eye(13);    % initial estimation error covariance

% process noise covariance
Q_k = diag([ ...
    0.01 0.01 0.01, ...        % position
    0.05 0.05 0.05, ...        % velocity
    1e-4 1e-4 1e-4 1e-4, ...   % quaternion
    0.01 0.01 0.01 ]);         % body rates

% measurement noise covariance
R_k = diag([ ...
    0.2^2 0.2^2 0.2^2, ...           % position noise ~ 0.2 m
    0.1^2 0.1^2 0.1^2, ...           % velocity noise ~ 0.1 m/s
    (deg2rad(2))^2 * ones(1,4), ...  % attitude noise ~2 deg
    (deg2rad(1))^2 * ones(1,3) ]);   % rate noise ~1 deg/s

sigma_meas = sqrt(diag(R_k));
v0 = sigma_meas .* randn(13,1);  % initial measurement noise
y_meas_0 = x_true + v0;          % initial measurement

% histories for plotting
x_true_hist = zeros(13, N_sim+1);
x_hat_hist  = zeros(13, N_sim+1);
y_meas_hist = zeros(13, N_sim+1);

x_err_true_hist = zeros(13, N_sim+1);
x_err_hat_hist  = zeros(13, N_sim+1);

u_hist     = zeros(4, N_sim);
u_raw_hist = zeros(4, N_sim);

x_true_hist(:,1)    = x_true;
x_hat_hist(:,1)     = x_hat;
y_meas_hist(:,1)    = y_meas_0;
x_err_true_hist(:,1)= x_true - x_ref;
x_err_hat_hist(:,1) = x_hat - x_ref;

% initialize previous control input
u_prev = U_eq;

% main simulation loop
for k = 1:N_sim
    t = (k-1)*Ts;

    % build noisy measurement
    v_k = sigma_meas .* randn(13,1);
    y_meas = x_true + v_k;

    % kalman prediction
    [x_pred, P_pred, auxKF] = kalman_filter_update( ...
        x_hat, P, u_prev, px4_config, Q_k, R_k, Ts);

    K = auxKF.K;    % Kalman gain
    x_pred = auxKF.x_pred; % predicted state (k+1|k)
    P_pred = auxKF.P_pred; % predicted covariance

    % kalman correction
    % H = I;
    innovation = y_meas - x_pred;
    x_hat = x_pred + K * innovation;
    P = (eye(13) - K) * P_pred;

    % normalize quaternion
    q_hat = x_hat(7:10);
    q_hat = q_hat / norm(q_hat);
    x_hat(7:10) = q_hat;

    % LQR control law
    x_err = x_hat - x_ref;
    [u_lqr, K_lqr, auxLQR] = lqr_controller(x_hat, x_ref, px4_config, Q_lqr, R_lqr);
    u_raw = u_lqr;

    % apply control input saturation
    u_sat = saturate_control(u_lqr, px4_config);

    % propagate true dynamics with RK4
    dyn_fun = @(t_local, x_local) drone_nonlinear_dynamics( ...
        t_local, x_local, u_sat, px4_config);

    x_true = RK4(dyn_fun, x_true, Ts, t);

    % normalize quaternion to avoid drift
    q_true = x_true(7:10);
    q_true = q_true / norm(q_true);
    x_true(7:10) = q_true;

    % log data
    x_true_hist(:,k+1)     = x_true;
    x_hat_hist(:,k+1)      = x_hat;
    y_meas_hist(:,k+1)     = y_meas;

    x_err_true_hist(:,k+1) = x_true - x_ref;
    x_err_hat_hist(:,k+1)  = x_hat - x_ref;

    u_hist(:,k)            = u_sat;
    u_raw_hist(:,k)        = u_raw;

    u_prev = u_sat;

end

n_samples    = 40;
sample_idx   = round(linspace(1, N_sim+1, n_samples));
sample_times = t_hist(sample_idx);

tau_max = 1.5;   % Nm , consistent with saturate_control

fprintf('\n=== LQG OFFLINE DEBUG SUMMARY ===\n');

for i = 1:n_samples
    idx = sample_idx(i);
    t   = sample_times(i);

    % true state at this time
    x_k_true = x_true_hist(:, idx);
    pos_true = x_k_true(1:3);
    vel_true = x_k_true(4:6);
    qvec_true = x_k_true(8:10);
    omega_true = x_k_true(11:13);

    % estimated state
    x_k_hat = x_hat_hist(:, idx);
    pos_hat = x_k_hat(1:3);
    qvec_hat = x_k_hat(8:10);
    omega_hat = x_k_hat(11:13);

    % position errors
    e_pos_true = pos_true - x_ref(1:3);
    e_pos_hat  = pos_hat  - x_ref(1:3);
    e_pos_true_norm = norm(e_pos_true);
    e_pos_hat_norm  = norm(e_pos_hat);

    % attitude error magnitudes (norm of vector part)
    att_err_true = norm(qvec_true);
    att_err_hat  = norm(qvec_hat);

    % control around this index
    idx_u   = min(max(idx-1,1), N_sim);
    u_sat_k = u_hist(:, idx_u);
    u_raw_k = u_raw_hist(:, idx_u);

    T_k   = u_sat_k(1);
    tau_k = u_sat_k(2:4);

    T_sat_flag   = abs(u_sat_k(1) - u_raw_k(1)) > 1e-6;
    tau_sat_flag = any(abs(u_raw_k(2:4)) > tau_max + 1e-6);

    fprintf(['t = %5.2fs | pos_true = [%6.3f, %6.3f, %6.3f] ', ...
             '||e_true|| = %6.3f | pos_hat = [%6.3f, %6.3f, %6.3f] ', ...
             '||e_hat|| = %6.3f | att_err_true = %7.4f | att_err_hat = %7.4f | ', ...
             'ω_true = [%7.4f, %7.4f, %7.4f] | ', ...
             'T = %6.2f N, τ = [%7.4f, %7.4f, %7.4f] Nm'], ...
             t, ...
             pos_true(1), pos_true(2), pos_true(3), e_pos_true_norm, ...
             pos_hat(1), pos_hat(2), pos_hat(3), e_pos_hat_norm, ...
             att_err_true, att_err_hat, ...
             omega_true(1), omega_true(2), omega_true(3), ...
             T_k, tau_k(1), tau_k(2), tau_k(3));

    if T_sat_flag
        fprintf(' [T_SAT]');
    end
    if tau_sat_flag
        fprintf(' [τ_SAT]');
    end

    fprintf('\n');
end

fprintf('=== END OF LQG SUMMARY ===\n\n');

%% Plots: true vs estimate vs reference

% positions
px_true = x_true_hist(1,:);
py_true = x_true_hist(2,:);
pz_true = x_true_hist(3,:);

px_hat  = x_hat_hist(1,:);
py_hat  = x_hat_hist(2,:);
pz_hat  = x_hat_hist(3,:);

figure;
subplot(3,1,1);
plot(t_hist, px_true, 'b', 'LineWidth', 1.5); hold on;
plot(t_hist, px_hat,  'r--', 'LineWidth', 1.2);
yline(x_ref(1), 'k:', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]'); ylabel('p_x [m]');
legend('true','estimate','ref');
title('Position p_x');

subplot(3,1,2);
plot(t_hist, py_true, 'b', 'LineWidth', 1.5); hold on;
plot(t_hist, py_hat,  'r--', 'LineWidth', 1.2);
yline(x_ref(2), 'k:', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]'); ylabel('p_y [m]');
legend('true','estimate','ref');
title('Position p_y');

subplot(3,1,3);
plot(t_hist, pz_true, 'b', 'LineWidth', 1.5); hold on;
plot(t_hist, pz_hat,  'r--', 'LineWidth', 1.2);
yline(x_ref(3), 'k:', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]'); ylabel('p_z [m]');
legend('true','estimate','ref');
title('Position p_z');

% attitude error (norm of vector part) true vs estimate
q_vec_true = x_true_hist(8:10,:);
q_vec_hat  = x_hat_hist(8:10,:);
q_vec_true_norm = vecnorm(q_vec_true, 2, 1);
q_vec_hat_norm  = vecnorm(q_vec_hat,  2, 1);

figure;
subplot(2,1,1);
plot(t_hist, x_true_hist(7,:), 'b', 'LineWidth', 1.5); hold on;
plot(t_hist, x_hat_hist(7,:),  'r--', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]'); ylabel('q_0');
legend('true','estimate');
title('Quaternion scalar part (q_0)');

subplot(2,1,2);
plot(t_hist, q_vec_true_norm, 'b', 'LineWidth', 1.5); hold on;
plot(t_hist, q_vec_hat_norm,  'r--', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]'); ylabel('||[q_1,q_2,q_3]||');
legend('true','estimate');
title('Attitude error magnitude (vector part norm)');

% Control inputs
figure;
plot(t_hist(1:end-1), u_hist(1,:), 'LineWidth', 1.2); hold on;
plot(t_hist(1:end-1), u_hist(2,:), 'LineWidth', 1.2);
plot(t_hist(1:end-1), u_hist(3,:), 'LineWidth', 1.2);
plot(t_hist(1:end-1), u_hist(4,:), 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('ctrl input components');
legend('T', '\tau_x', '\tau_y', '\tau_z');
title('LQG control inputs (offline)');

% Position error norms: true vs estimated
pos_err_true = vecnorm(x_true_hist(1:3,:) - x_ref(1:3), 2, 1);
pos_err_hat  = vecnorm(x_hat_hist(1:3,:)  - x_ref(1:3), 2, 1);

figure;
plot(t_hist, pos_err_true, 'b', 'LineWidth', 1.5); hold on;
plot(t_hist, pos_err_hat,  'r--', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('||e_{pos}|| [m]');
legend('true state error','estimated state error');
title('Position tracking error: true vs estimated');
