% filename: MATLAB/GNC_Algorithms/Control/NMPC/run_P07_NMPC_offline.m
%
% Problem 7: Pure NMPC trajectory tracking (offline, no PX4)
% Closed-loop: x_{k+1} = f_d(x_k, u_k), u_k from NMPC
%
% Outputs: plots for position vs reference, tracking error, and control inputs.

function run_P07_NMPC_offline()

    % ---------------------------------------------------------------------
    % 1) Model / NMPC setup
    % ---------------------------------------------------------------------
    px4_config = get_x500_params();   % just for physical params (m, g, inertia, etc.)

    Ts          = 0.1;       % sample time [s] for NMPC + simulation
    N_horizon   = 20;        % prediction horizon length
    T_final     = 60.0;      % total simulation time [s]

    nmpc = p07_setup_nmpc(px4_config, Ts, N_horizon);

    n_states    = nmpc.n_states;
    n_controls  = nmpc.n_controls;

    % Trajectory configuration (circle)
    traj_cfg.R      = 2.0;       % radius [m]
    traj_cfg.V      = 0.8;       % tangential speed [m/s]
    traj_cfg.center = [0; 2];    % [cx; cy]
    traj_cfg.z0     = -2.0;      % altitude [m]

    % Initial state
    x = zeros(n_states,1);
    x(1:3)   = [0; 0; 0];        % start near origin
    x(7)     = 1.0;              % quaternion = [1; 0; 0; 0]
    x(8:10)  = 0.0;

    % Hover input for initialization
    m = px4_config.m; 
    g = px4_config.g;
    U_eq = [m*g; 0; 0; 0];

    % previous input (for Δu penalty)
    u_prev = U_eq;

    % ---------------------------------------------------------------------
    % 2) Simulation storage
    % ---------------------------------------------------------------------
    n_steps = round(T_final / Ts);

    t_vec       = (0:n_steps) * Ts;            % time stamps (states)
    t_mid       = (0:n_steps-1) * Ts;          % time stamps (inputs / ref)

    X_hist      = zeros(n_states, n_steps+1);  % states
    X_ref_hist  = zeros(n_states, n_steps);    % reference (current ref each step)
    U_hist      = zeros(n_controls, n_steps);  % applied inputs

    X_hist(:,1) = x;

    % ---------------------------------------------------------------------
    % 3) Closed-loop NMPC simulation
    % ---------------------------------------------------------------------
    fprintf('Starting offline NMPC simulation (pure closed-loop)...\n');

    for k = 1:n_steps
        t = (k-1)*Ts;

        % -------------------------------------------------------------
        % Build reference trajectory over horizon using traj_circle
        % -------------------------------------------------------------
        Xref_hor = zeros(n_states, N_horizon);
        for j = 1:N_horizon
            t_j = t + (j-1)*Ts;
            Xref_hor(:,j) = traj_circle(t_j, traj_cfg);
        end

        % Current reference for logging / error
        x_ref_curr = Xref_hor(:,1);

        % -------------------------------------------------------------
        % NMPC step: compute optimal control sequence, apply first input
        % -------------------------------------------------------------
        [u_nmpc, aux_nmpc] = p07_mpc_step(x, Xref_hor, u_prev, nmpc);

        % Saturate control inputs for realism
        u_sat = saturate_control(u_nmpc, px4_config);

        % Update previous input for Δu cost next step
        u_prev = u_sat;

        % -------------------------------------------------------------
        % Plant update (nonlinear dynamics + RK4)
        % -------------------------------------------------------------
        dynamics_func = @(t_local, x_local) drone_nonlinear_dynamics( ...
            t_local, x_local, u_sat, px4_config);

        x_next = RK4(dynamics_func, x, Ts, t);

        % Normalize quaternion
        q_norm = norm(x_next(7:10));
        if q_norm > 1e-6
            x_next(7:10) = x_next(7:10) / q_norm;
        else
            % fallback to identity if something goes wrong
            x_next(7:10) = [1; 0; 0; 0];
        end

        % -------------------------------------------------------------
        % Store data
        % -------------------------------------------------------------
        X_hist(:,k+1)    = x_next;
        X_ref_hist(:,k)  = x_ref_curr;
        U_hist(:,k)      = u_sat;

        % move state forward
        x = x_next;
    end

    fprintf('Offline NMPC simulation finished.\n');

    % ---------------------------------------------------------------------
    % 4) Plot results
    % ---------------------------------------------------------------------
    plot_P07_nmpc_offline_results(t_vec, t_mid, X_hist, X_ref_hist, U_hist);
end
