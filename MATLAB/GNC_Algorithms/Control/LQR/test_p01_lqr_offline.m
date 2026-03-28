% filename: MATLAB/GNC_Algorithms/Control/LQR/test_p01_lqr_offline.m

% offline test of LQR controller for quadcopter hover

clear; clc; close all;

% load quadcopter parameters
px4_config = get_x500_params();

% setup LQR 

x_ref = zeros(13,1);
x_ref(7) = 1.0;

% --- Q weights ---
Q_pos = diag([ 4,  4,  8]);          % px, py, pz  (pz more important)
Q_vel = diag([ 1,  1,  2]);          % vx, vy, vz  (vz a bit more)
Q_q   = diag([ 40,  20,  20,  20]);  % q0, q1, q2, q3
Q_om  = diag([ 2,   2,   2]);        % p, q, r

Q = blkdiag(Q_pos, Q_vel, Q_q, Q_om);

% --- R weights ---

% Make thrust relatively cheap, torques expensive
R = diag([0.5,  15,  15,  15]);   % [T, τx, τy, τz]

% feedforward control input
m = px4_config.m;
g = px4_config.g;
U_eq = [m * g; 0; 0; 0];

% compute LQR gain at equilibrium
x_eq = zeros(13,1);
x_eq(7) = 1.0;   % hover attitude

[A_eq, B_eq] = drone_linear_dynamics(x_eq, U_eq, px4_config);

K_eq = lqr(A_eq, B_eq, Q, R);

% initial state
x0 = zeros(13,1);

% start with a small position and attitude error
x0(1:3) = [5.0; -0.5; -2.0];       % [px, py, pz]
q_init  = [1.0; 0; 0; 0.0];   % small attitude error
q_init  = q_init / norm(q_init);   % normalize quaternion
x0(7:10) = q_init;

x = x0;

% simulation settings
Ts    = 0.01;             % [s] control update and simulation timestep
T_sim = 5.0;            % [s] total simulation time
N_sim = round(T_sim / Ts);

t_hist     = (0:N_sim)*Ts;
x_hist     = zeros(13, N_sim+1);
x_err_hist = zeros(13, N_sim+1);
u_hist     = zeros(4,  N_sim);
u_raw_hist = zeros(4,  N_sim);   % raw LQR (before saturation)

% initial storage
x_hist(:,1)     = x;
x_err_hist(:,1) = x - x_ref;


% main simulation loop
for k = 1:N_sim
    t = (k-1)*Ts;

    % LQR control law
    x_err = x - x_ref;
    [u_lqr, K, aux] = lqr_controller(x, x_ref, px4_config, Q, R);

    %u_lqr = U_eq - K * (x - x_ref);

    % % LQR control law (fixed)
    % x_err = x - x_ref;
    % u_lqr = U_eq - K_eq * x_err;

    % apply control input saturation
    u_sat = saturate_control(u_lqr, px4_config);

    % propagate dynamics with RK4
    dyn_fun = @(t_local, x_local) drone_nonlinear_dynamics(...
        t_local, x_local, u_sat, px4_config);

    x = RK4(dyn_fun, x, Ts, t);

    % normalize quaternion to avoid drift
    q = x(7:10);
    q = q / norm(q);
    x(7:10) = q;

    % save the history
    x_hist(:,k+1)     = x;
    u_raw_hist(:,k) = u_lqr;
    u_hist(:,k)     = u_sat;
    x_err_hist(:,k+1) = x_err;
end

% rext summary at selected times (for debugging)

% Pick a few sample indices across the simulation
% (here: start, 1/3, 2/3, end)
n_samples    = 40;
sample_idx   = round(linspace(1, N_sim+1, n_samples));  % indices into x_hist
sample_times = t_hist(sample_idx);

fprintf('\n=== LQR OFFLINE DEBUG SUMMARY ===\n');

% torque saturation threshold (same as online log)
tau_max = 1.5;   % [Nm] (adjust if your saturate_control uses something else)

for i = 1:n_samples
    idx_x = sample_idx(i);
    t     = sample_times(i);

    % State at this time
    x_k   = x_hist(:, idx_x);
    pos   = x_k(1:3);
    vel   = x_k(4:6);
    q0_k  = x_k(7);
    qvec  = x_k(8:10);
    omega = x_k(11:13);

    % Position error (relative to reference)
    e_pos = pos - x_ref(1:3);
    e_pos_norm = norm(e_pos);

    % Attitude error magnitude (norm of vector part)
    att_err = norm(qvec);

    % Control at (approximately) this time
    idx_u     = min(max(idx_x-1,1), N_sim);
    u_sat_k   = u_hist(:, idx_u);      % saturated
    u_raw_k   = u_raw_hist(:, idx_u);  % raw LQR

    T_k       = u_sat_k(1);
    tau_k     = u_sat_k(2:4);

    % --- Saturation flags (like online version) ---
    T_sat_flag   = abs(u_sat_k(1) - u_raw_k(1)) > 1e-6;
    tau_sat_flag = any(abs(u_raw_k(2:4)) > tau_max + 1e-6);

    % Base line
    fprintf(['t = %5.2fs | pos = [%6.3f, %6.3f, %6.3f] ', ...
             'err = [%6.3f, %6.3f, %6.3f] (||e|| = %6.3f) | ' , ...
             'vel = [%6.3f, %6.3f, %6.3f] | ' , ...
             'att_err = %7.4f | ' , ...
             'ω = [%7.4f, %7.4f, %7.4f] | ' , ...
             'T = %6.2f N, τ = [%7.4f, %7.4f, %7.4f] Nm'], ...
             t, ...
             pos(1), pos(2), pos(3), ...
             e_pos(1), e_pos(2), e_pos(3), e_pos_norm, ...
             vel(1), vel(2), vel(3), ...
             att_err, ...
             omega(1), omega(2), omega(3), ...
             T_k, tau_k(1), tau_k(2), tau_k(3));

    % Append saturation markers like in online log
    if T_sat_flag
        fprintf(' [T_SAT]');
    end
    if tau_sat_flag
        fprintf(' [τ_SAT]');
    end

    fprintf('\n');
end

fprintf('=== END OF SUMMARY ===\n\n');

% plot results

% position
px = x_hist(1,:);
py = x_hist(2,:);
pz = x_hist(3,:);

% attitude (quaternion)
q0_hist = x_hist(7,:);
q_vec = x_hist(8:10,:);
q_vec_norm = vecnorm(q_vec, 2, 1);

figure;
subplot(3,1,1);
plot(t_hist, px, 'LineWidth', 1.5); hold on;
yline(x_ref(1), '--', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('p_x');
title('Position p_x vs reference');

subplot(3,1,2);
plot(t_hist, py, 'LineWidth', 1.5); hold on;
yline(x_ref(2), '--', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('p_y [m]');
title('Position p_y');

subplot(3,1,3);
plot(t_hist, pz, 'LineWidth', 1.5); hold on;
yline(x_ref(3), '--', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('p_z [m]');
title('Position p_z');

figure;
subplot(2,1,1);
plot(t_hist, q0_hist, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('q_0');
title('Quaternion scalar part (q_0)');

subplot(2,1,2);
plot(t_hist, q_vec_norm, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('||[q_1,q_2,q_3]||');
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
title('LQR control inputs (offline)');
