% filename: MATLAB/GNC_Algorithms/Control/run_P05_LMPC_traj_multi.m

function run_P05_LMPC_traj_multi()

    config = px4_get_config();
    client = px4_connect(config.ip_address, config.port);
    
    px4_config = get_x500_params(); % quadcopter physical properties
    start_time = tic;


    % Select CONTROLLER mode
    CONTROL_MODE = 'position';

    % set take off altitude
    config.takeoff_altitude = -5.0;

    % parameters
    dt_dyn = 1/3;
    flight_duration = 60;
    Ts = dt_dyn;

    N_horizon = 15;  % prediction horizon
    k_lookahead = 6; 

    % trajectory
    refcfg.z0  = config.takeoff_altitude;     % constant altitude 
    refcfg.p0  = [0; 0; refcfg.z0];           % start at (0,0,z0)
    refcfg.v   = [0.1; 0; 0];                 % 0.1 m/s along +x
    refcfg.yaw = 0;


    % control input feedforward
    m = px4_config.m; 
    g = px4_config.g;
    U_eq = [m * g; 0; 0; 0];


    % linearized for LMPC
    x_eq        = zeros(13,1);
    x_eq(3)     = config.takeoff_altitude;
    x_eq(7)     = 1.0;

    [A, B] = drone_linear_dynamics(x_eq, U_eq, px4_config);
    Ad = eye(size(A)) + A * Ts;
    Bd = B * Ts;

    % LMPC setup
    lmpc = p05_setup_lmpc_multi_traj(px4_config, Ts, N_horizon, Ad, Bd);

    % set u_prev for penal
    u_prev = U_eq;

    fprintf('Starting Simulation...\n');
    fprintf('Control mode: %s\n', CONTROL_MODE);

    % arm the drone for manual control
    px4_enter_offboard_mode(client, config);
    pause(2);

    px4_arm_drone(client, config);
    pause(2);
    
    px4_takeoff(client, config);
    pause(15);
    
    % Verify takeoff 
    telemetry = px4_get_telemetry(client, config);
    current_alt = -telemetry.odometry.position(3);
    
    if current_alt < 1.5
        error('Takeoff failed - altitude: %.1fm', current_alt);
    end
    
    fprintf('Takeoff complete at %.1fm\n', current_alt);
    
    % switch to control mode
    px4_switch_control_mode(client, CONTROL_MODE, config);
    pause(2);  % Allow mode transition
    
    fprintf('Starting control...\n');
    log_data = initialize_logging();
    log_data.N_horizon   = N_horizon;
    log_data.k_lookahead = k_lookahead;

     % simulation loop
    fprintf('Starting the P05 - LMPC simulation...\n');
    t=0;

    while t < flight_duration
        loop_start = tic;
        
        % get telemetry data
        telemetry = px4_get_telemetry(client, config);
        
        % get current state vector
        x_curr = state_vec(telemetry);
        x_curr(7:10) = x_curr(7:10) / norm(x_curr(7:10));
        
        % build reference trajectory over horizon
        Xref_hor = zeros(13, N_horizon);
        for j = 1:N_horizon
            t_j = t + (j-1)*Ts;
            Xref_hor(:,j) = traj_line(t_j, refcfg);
        end

        x_ref_curr = Xref_hor(:,1);
        x_err = x_curr - x_ref_curr;
        
        % LMPC step: compute optimal control sequence, apply first input
        t_mpc_start = tic;
        [u_lmpc, aux_lmpc] = p08b_mpc_step_multi_traj(x_curr, Xref_hor, u_prev, lmpc);
        t_mpc = toc(t_mpc_start);
        
        % Saturate control inputs for realism
        u_sat = saturate_control(u_lmpc, px4_config);
        
        % Update previous input for deltau cost in the next step
        u_prev = u_sat;
        
        idx_look = min(k_lookahead, lmpc.N + 1);   % safety
        x_sp = aux_lmpc.X_opt(:, idx_look);        % [13x1] predicted state
        x_sp(7:10) = x_sp(7:10) / norm(x_sp(7:10));
        yaw_sp = 0;
        
        thrust_cmd = u_sat(1);
        tau_cmd    = u_sat(2:4);

                % Send control commands based on selected mode
        roll_des = 0; pitch_des = 0; yaw_des = 0; angle_limited = false;  % Initialize for all modes
        
        switch CONTROL_MODE
            case 'position'
                %px4_send_trajectory(client, x_next(1), x_next(2), x_next(3), 0, config);
                px4_send_trajectory(client, x_sp(1), x_sp(2), x_sp(3), yaw_sp, config);
                
            case 'attitude'
                [q_desired, roll_des, pitch_des, yaw_des, angle_limited] = saturate_attitude(x_next, deg2rad(15));
                px4_send_attitude_setpoint(client, thrust_cmd, q_desired(1), q_desired(2), q_desired(3), q_desired(4), config);
                
            case 'rates'
                % Extract predicted body rates
                omega_des = x_next(11:13);
                
                % Saturate rates for safety
                max_rate = deg2rad(100);
                omega_des = max(-max_rate, min(max_rate, omega_des));

                px4_send_rate_setpoint(client, thrust_cmd, omega_des(1), omega_des(2), omega_des(3), config);
   
            otherwise
                error('Invalid CONTROL_MODE: %s. Must be position, attitude, or rates', CONTROL_MODE);
        end

        % REMOVE or MODIFY the lines below to display the current status of the simulation
       if mod(round(t/dt_dyn), 25) == 0
            fprintf('t=%.1fs | pos=[%.2f,%.2f,%.2f] err=[%.2f,%.2f,%.2f](%.2fm) | vel=[%.2f,%.2f,%.2f] | ', ...
                    t, x_curr(1), x_curr(2), x_curr(3), ...
                    x_err(1), x_err(2), x_err(3), norm(x_err(1:3)), ...
                    x_curr(4), x_curr(5), x_curr(6));
            
            switch CONTROL_MODE
                case 'attitude'
                    fprintf('att=[%.1f,%.1f,%.1f]° ', ...
                            rad2deg(roll_des), rad2deg(pitch_des), rad2deg(yaw_des));
                    if angle_limited, fprintf('[ANG_LIM] '); end
                    
                case 'rates'
                    fprintf('ω_cmd=[%.3f,%.3f,%.3f] ω_curr=[%.3f,%.3f,%.3f] ', ...
                            omega_des(1), omega_des(2), omega_des(3), ...
                            x_curr(11), x_curr(12), x_curr(13));
                    if any(abs(x_next(11:13)) > deg2rad(100)), fprintf('[RATE_SAT] '); end
            end
            
            fprintf('| T=%.1fN τ=[%.3f,%.3f,%.3f]Nm', ...
                    u_sat(1), tau_cmd(1), tau_cmd(2), tau_cmd(3));
            if u_sat(1) ~= u_lmpc(1), fprintf(' [T_SAT]'); end
            if any(abs(u_lmpc(2:4)) > 1.5), fprintf(' [τ_SAT]'); end
            fprintf('\n');

            fprintf('| T_virt=%.1fN τ_virt=[%.3f,%.3f,%.3f]Nm | N=%d k_look=%d t_MPC=%.3fs\n', ...
                thrust_cmd, tau_cmd(1), tau_cmd(2), tau_cmd(3), ...
                lmpc.N, k_lookahead, t_mpc);
        end

        % Logging and Timing
        log_data = update_log(log_data, t, x_curr, x_err, u_sat);

        % Add reference and predicted state for this sample
        i_log = log_data.index - 1;      % last sample index used in update_log (because i is updated already)
        
        log_data.ref(:, i_log)  = x_ref_curr;
        
        log_data.pred(:, i_log) = x_sp;

        % MPC solver time
        log_data.mpc_time(i_log) = t_mpc;

        t = t + dt_dyn;
        elapsed = toc(loop_start);

        % Store CPU time per control step
        log_data.step_time(i_log) = elapsed;  % seconds

        if elapsed < dt_dyn
            pause(dt_dyn - elapsed);
        end
    end
    
    save_log_data(log_data, 'log_p05_lmpc.mat');
    plot_tracking_results('log_p05_lmpc.mat', 'P05 - LMPC Tracking (Multi Shooting)');

    % px4_initiate_landing(client, config);
    % pause(5);
    % px4_disarm_drone(client, config);
    reinitial_x500("tracking");

end

function [q_desired, roll_des, pitch_des, yaw_des, angle_limited] = saturate_attitude(x_next, max_tilt_angle)
    % Extract predicted quaternion as desired attitude
    q_desired = x_next(7:10);
    
    % Apply safety limits on tilt angles
    euler_des = quat2eul(q_desired', 'ZYX');  % [yaw, pitch, roll]
    yaw_des = euler_des(1);
    pitch_des = euler_des(2);
    roll_des = euler_des(3);
    
    % Check if saturation is needed, based on max_tilt_angle
    angle_limited = false;
    if abs(roll_des) > max_tilt_angle || abs(pitch_des) > max_tilt_angle
        angle_limited = true;
        roll_des = max(-max_tilt_angle, min(max_tilt_angle, roll_des));
        pitch_des = max(-max_tilt_angle, min(max_tilt_angle, pitch_des));
        q_desired = eul2quat([yaw_des, pitch_des, roll_des], 'ZYX')';
    end
end