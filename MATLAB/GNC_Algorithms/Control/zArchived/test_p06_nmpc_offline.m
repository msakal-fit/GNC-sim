% filename: MATLAB/GNC_Algorithms/Control/NMPC/p06_test_nmpc_offline.m
%
% Offline test of NMPC hover (Problem 6)
% - no PX4, just MATLAB + RK4 integration
% - uses the same nonlinear dynamics as in the main project
%
% Goal:
%   Start from some initial position/orientation,
%   use NMPC to stabilize altitude and attitude around a hover state.

clear; clc; close all;

%% 1) Load quadcopter parameters and build NMPC

px4_config = get_x500_params();

Ts = 0.05;     % [s] NMPC/control sample time (can try 0.1 too)
N  = 20;       % horizon length  (N * Ts seconds)

nmpc = p06_setup_mpc(px4_config, Ts, N);

%% 2) Define reference hover state

% State ordering: [ px py pz  vx vy vz  q0 q1 q2 q3  p q r ]'
x_ref = zeros(13,1);
x_ref(3) = -5.0;     % desired altitude

% Level attitude: q0 = 1, [q1 q2 q3] = 0
x_ref(7) = 1.0;     % q0
x_ref(8:10) = 0.0;  % q1, q2, q3

% Velocities and angular rates already zero

%% 3) Initial state

x0 = zeros(13,1);

% Start a bit away from the target altitude and with a slight tilt
x0(1:3) = [0; 0; 0];       % start at "ground" level
x0(7)   = 1.0;             % q0 ~ 1
x0(8:10) = [0.1; -0.1; 0]; % small initial attitude error
% velocities and angular rates = 0

x = x0;

%% 4) Simulation settings

T_sim  = 10.0;                 % [s] total simulation time
N_sim  = round(T_sim / Ts);    % number of simulation steps

x_hist = zeros(13, N_sim+1);
u_hist = zeros(4,  N_sim);
t_hist = (0:N_sim)*Ts;

x_hist(:,1) = x;

%% 5) Main simulation loop

for k = 1:N_sim
    t = (k-1)*Ts;

    % --- 5.1 Call NMPC to get control ---
    [u_cmd, aux] = p06_mpc_step(x, x_ref, nmpc); % NMPC step
    u_hist(:,k)  = u_cmd;

    % --- 5.2 Propagate dynamics with RK4 ---
    % Define a local function handle for current control
    dyn_fun = @(t_local, x_local) drone_nonlinear_dynamics( ...
        t_local, x_local, u_cmd, px4_config);

    x = RK4(dyn_fun, x, Ts, t);

    % Normalize quaternion to avoid drift
    q = x(7:10);
    q = q / norm(q);
    x(7:10) = q;

    % Save state
    x_hist(:,k+1) = x;
end

%% 6) Simple plots: altitude & attitude

pz      = x_hist(3,:);          % altitude (check sign convention)
q0_hist = x_hist(7,:);
q_vec   = x_hist(8:10,:);       % [q1; q2; q3]
q_vec_norm = vecnorm(q_vec, 2, 1);  % norm of vector part

figure;
subplot(3,1,1);
plot(t_hist, pz, 'LineWidth', 1.5); hold on;
yline(x_ref(3), '--', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('p_z');
title('Altitude (p_z) vs reference');

subplot(3,1,2);
plot(t_hist, q0_hist, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('q_0');
title('Quaternion scalar part (q_0)');

subplot(3,1,3);
plot(t_hist, q_vec_norm, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('||[q_1,q_2,q_3]||');
title('Attitude error magnitude (vector part of quaternion)');

figure;
plot(t_hist(1:end-1), u_hist(1,:), 'LineWidth', 1.2); hold on;
plot(t_hist(1:end-1), u_hist(2,:), 'LineWidth', 1.2);
plot(t_hist(1:end-1), u_hist(3,:), 'LineWidth', 1.2);
plot(t_hist(1:end-1), u_hist(4,:), 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('u components');
legend('T', '\tau_x', '\tau_y', '\tau_z');
title('NMPC control inputs');
