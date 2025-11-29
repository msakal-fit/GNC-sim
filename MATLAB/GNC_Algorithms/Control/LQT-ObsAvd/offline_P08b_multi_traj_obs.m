% filename: MATLAB/GNC_Algorithms/Control/LQT-ObsAvd/offline_P08b_multi_traj_obs.m

function offline_P08b_multi_traj_obs()

    %quadcopter parameters
    px4_config = get_x500_params();

    % NMPC setup
    Ts = 1/3;                            % sampling time
    N_horizon = 60;                      % prediction horizon length
    T_sim = 60;                          % total simulation time

    % build NPMC with obstacle as constraints
    nmpc = p08b_setup_mpc_multi_tracj_obs(px4_config, Ts, N_horizon);
    %nmpc = p08b_setup_mpc_multi_tracj(px4_config, Ts, N_horizon); 

    % setup reference trajectory
    refcfg = struct();
    refcfg.p0  = [0; 0; -5];    % start at (0,0,-5)
    refcfg.v   = [0.3; 0; 0];   % 0.3 m/s along xplus direction
    refcfg.yaw = 0;

    % initialate state at start of sim
    x = zeros(13,1);
    x((1:3)) = refcfg.p0;
    x(7) = 1;   % initial quat

    % initial input
    u_prev = nmpc.U_ref;

    % logging
    log_data = initialize_logging();

    t = 0.0;
    step = 1;

    PRINT_INVEVAL = 2.0;
    next_print_time = 0.0;

    while t <= T_sim

        % build reference from current time
        Xref_hor = zeros(13, N_horizon);
        for k = 1:N_horizon
            tk = t + (k-1)*Ts;
            Xref_hor(:,k) = traj_line(tk, refcfg);
        end

        % current reference (for logging & debug)
        x_ref_curr = traj_line(t, refcfg);

        tic_step = tic;

        % solve NMPC
        [u_mpc, aux] = p08b_mpc_step_multi_traj(x, Xref_hor, u_prev, nmpc);
        u_prev = u_mpc;

        dt_step = toc(tic_step);       % CPU time for this MPC call

        % propagate dynamics
        x_next = full(nmpc.F_RK4(x, u_mpc));
        x_next = x_next(:);

        % log data
        % compute error for logging / debug
        x_err    = x - x_ref_curr;
        pos      = x(1:3);
        pos_ref  = x_ref_curr(1:3);
        pos_err  = pos - pos_ref;
        err_norm = norm(pos_err);

        % --- Debug print (similar style to run_P08b) ---
        T_val   = u_mpc(1);
        tau_val = u_mpc(2:4);

        if t >= next_print_time

            % distance to obstacle center (3D)
            c3d  = nmpc.obs_center3d;  % [x;y;z]
            dist = norm([x(1) - c3d(1);
                        x(2) - c3d(2);
                        x(3) - c3d(3)]);


            fprintf(['t=%.2fs | pos=[%.2f,%.2f,%.2f] err=[%.2f,%.2f,%.2f](%.2fm) ', ...
                        '| dist_to_obs=%.2f (R_safe=%.2f) ', ...
                        '| T=%.2fN tau=[%.2f,%.2f,%.2f] | dt_mpc=%.4fs\n'], ...
                        t, ...
                        pos(1), pos(2), pos(3), ...
                        pos_err(1), pos_err(2), pos_err(3), err_norm, ...
                        dist, nmpc.R_safe, ...
                        T_val, tau_val(1), tau_val(2), tau_val(3), ...
                        dt_step);
                
            next_print_time = next_print_time + PRINT_INVEVAL;

        end

        % log data (includes ref + CPU time)
        log_data = update_log(log_data, t, x, x_err, u_mpc);

        % Add reference and predicted state for this sample
        i_log = log_data.index - 1;      % last sample index used in update_log (because i is updated already)
        log_data.ref(:, i_log)  = x_ref_curr;
        log_data.pred(:, i_log) = x_next;
        log_data.step_time(i_log) = dt_step;

        % update for next iteration
        x = x_next;
        t = t + Ts;
        step = step + 1;
    end

    % save log
    log_filename = 'log_offline_P08b_multi_traj_obs.mat';
    save_log_data(log_data, log_filename);

    % --- Call the tracking plot function for full diagnostics ---
    plot_tracking_results(log_filename, 'NMPC multi-shooting + obstacle (offline)');

    % quick XY plot with obstacle
    L        = log_data;
    X_states = L.state;   % 13 x N

    figure; hold on; axis equal;
    plot(X_states(1,:), X_states(2,:), 'LineWidth', 1.5);

    % draw obstacle keep-out circle
    c  = nmpc.obs_center3d;
    R  = nmpc.R_safe;
    th = linspace(0, 2*pi, 200);
    plot(c(1) + R*cos(th), c(2) + R*sin(th), '--');

    xlabel('x [m]');
    ylabel('y [m]');
    legend('quad path', 'obstacle + x m margin', 'Location', 'best');
    grid on;
    title('Offline NMPC trajectory with obstacle');

    % XZ cross-section with 3D obstacle sphere
    figure; hold on; axis equal;
    plot(X_states(1,:), X_states(3,:), 'LineWidth', 1.5);

    c3d = nmpc.obs_center3d;
    R   = nmpc.R_safe;
    th  = linspace(0, 2*pi, 200);
    plot(c3d(1) + R*cos(th), c3d(3) + R*sin(th), '--');

    xlabel('x [m]');
    ylabel('z [m]');
    legend('quad path', 'obstacle + margin (x-z cross-section)', 'Location', 'best');
    grid on;
    title('X-Z cross-section (vertical avoidance)');


    % use NMPC generated trajectory as ref for LQT run


    % make sure that the timestep is the same
    t_ref  = L.time(:);     % N x 1
    X_ref  = L.state;       % 13 x N
    U_refN = L.control;     % 4 x N
    Ts_ref = mean(diff(t_ref));

    ref_traj = struct();
    ref_traj.t  = t_ref;    % time stamps
    ref_traj.Ts = Ts_ref;   % sample time
    ref_traj.X  = X_ref;    % 13 x N reference states for LQT
    ref_traj.U  = U_refN;

    ref_filename = 'ref_traj_P08b_obs_for_LQT.mat';
    save(ref_filename, 'ref_traj');
    fprintf('LQT reference trajectory saved to: %s\n', ref_filename);

end