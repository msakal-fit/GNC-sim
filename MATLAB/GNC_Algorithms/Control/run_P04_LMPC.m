% filename: MATLAB/GNC_Algorithms/Control/run_P04_LMPC.m

function run_P04_LMPC()

    % Establish connection between MATLAB and simulator
    config = px4_get_config();
    client = px4_connect(config.ip_address, config.port);
    
    px4_config = get_x500_params(); % quadcopter physical properties
    start_time = tic;
    
    % CONTROLLER SELECTION
    CONTROL_MODE = 'position';  % Options: 'position', 'attitude', 'rates'
    
    % MODIFY TAKEOFF ALTITUDE, IF NEEDED.
    config.takeoff_altitude = -5.0;

    % parameters
    switch CONTROL_MODE
        case 'position'
            dt_dyn = 1/3;
            flight_duration = 100;
        case 'attitude'
            dt_dyn = 1/250;
            flight_duration = 10;
        case 'rates'
            dt_dyn = 1/250;
            flight_duration = 15;
    end
    
    % TARGET/REFERENCE STATES
    x_target = [ ...
        3; 0; -1; ...  % position
        0; 0; 0; ...  % velocity
        1; 0; 0; 0; ...  % quaternion
        0; 0; 0];  % body rates
    
    % FEEDFORWARD CONTROL INPUT
    m = px4_config.m; g = px4_config.g;
    U_eq = [m * g; 0; 0; 0];

    % --- LMPC parameters ---
    mpc = struct();
    mpc.N = 10;     % prediction horizon (10 steps * 1/3 s = ~3.3s lookahead)

    % State and input weights
    Q_pos  = 10.0 * eye(3);
    Q_vel  = 1.0 * eye(3);
    Q_quat = 2.0 * eye(4);
    Q_omg  = 0.5 * eye(3);

    mpc.Q = blkdiag(Q_pos, Q_vel, Q_quat, Q_omg);


    % Input weight
    % mpc.R = diag([1e-4, 1e-4, 1e-4, 1e-4]);

    % Terminal weight: same as Q for simplicity
    mpc.P = mpc.Q;

    % State bounds 
    % % [px; py; pz; vx; vy; vz; q0; q1; q2; q3; p; q; r]
    % x_max = [ 3;  3;  0.5;  2;  2;  2;  1;  1;  1;  1;  2;  2;  2];
    % x_min = -x_max;
    % mpc.x_min = x_min;
    % mpc.x_max = x_max;

    % --- State bounds in *error coordinates* ---
    % [e_px; e_py; e_pz; e_vx; e_vy; e_vz; e_q0; e_q1; e_q2; e_q3; p; q; r]

    pos_err_max = [ 5;  5;  6];    % allow up to 5 m horizontal, 6 m vertical error
    vel_err_max = [ 3;  3;  3];    % up to 3 m/s error
    quat_err_max = [ 1.5; 1.5; 1.5; 1.5];  % pretty loose, since we renormalize
    rate_max     = [ 4; 4; 4];     % rad/s, loose

    x_max_err = [pos_err_max;
                vel_err_max;
                quat_err_max;
                rate_max];

    x_min_err = -x_max_err;

    mpc.x_min = x_min_err;
    mpc.x_max = x_max_err;

    % Input bounds for [T; tau_x; tau_y; tau_z]
    thrust_hover = m * g;
    dT_max   = 0.5 * thrust_hover;    % +/-50% of hover
    tau_max  = 3 ;
    mpc.u_min = [-dT_max; -tau_max; -tau_max; -tau_max];
    mpc.u_max = [ dT_max;  tau_max;  tau_max;  tau_max];

    % --- Build linear model around hover once
    % Hover equilibrium:
    x_eq = x_target;                % hover at origin, level
    u_eq = [thrust_hover; 0; 0; 0]; % hover thrust, zero moments

    [A_c, B_c] = drone_linear_dynamics(x_eq, u_eq, px4_config);

    % Discretize (simple forward Euler)
    A_d = eye(size(A_c)) + dt_dyn * A_c;
    B_d = dt_dyn * B_c;

    % Input weighting: make sure R matches input dimension nu
    [~, nu] = size(B_d);
    mpc.R = 1e-4 * eye(nu);

    % Store for access in loop
    mpc.A_d = A_d;
    mpc.B_d = B_d;
    mpc.u_eq = u_eq;
    
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
    
    fprintf('Starting manual control...\n');
    log_data = initialize_logging();

     % simulation loop
    fprintf('Starting LMPC point stabilization (position mode)...\n');
    t = 0;
    
    u_prev = U_eq;
    
    while t < flight_duration
        loop_start = tic;
        
        % get telemetry data
        telemetry = px4_get_telemetry(client, config);
        
        % get current state vector
        x_curr = state_vec(telemetry);
        x_curr(7:10) = x_curr(7:10) / norm(x_curr(7:10));
        
        
        % IMPLEMENT YOUR CONTROLLER HERE
        % 1) Compute the error state (relative to target)
        x_err     = x_curr - x_target;        % we regulate to the origin hover

        % 2) LMPC step (solve QP for delta-u around hover)
        [u_delta, info_mpc] = lmpc_step(x_err, mpc.A_d, mpc.B_d, mpc);

        % 3) Shift back to absolute input: u = u_eq + delta_u
        u_cmd = mpc.u_eq + u_delta;

        % 4) Enforce hard saturations as a final safety layer
        u_cmd = saturate_control(u_cmd, px4_config);
       
        % Nonlinear dynamics integration
        dynamics_func = @(t, x) drone_nonlinear_dynamics(t, x, u_cmd, px4_config);
        x_next = RK4(dynamics_func, x_curr, dt_dyn, t);
        
        % Normalize the predicted quaternion
        x_next(7:10) = x_next(7:10) / norm(x_next(7:10));
        
        thrust_cmd = u_cmd(1);
        tau_cmd = u_cmd(2:4);
        
        % Send control commands based on selected mode
        roll_des = 0; pitch_des = 0; yaw_des = 0; angle_limited = false;  % Initialize for all modes
        
        switch CONTROL_MODE
            case 'position'
                px4_send_trajectory(client, x_next(1), x_next(2), x_next(3), 0, config);
                
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
                    u_cmd(1), tau_cmd(1), tau_cmd(2), tau_cmd(3));
            if u_cmd(1) ~= mpc.u_eq(1), fprintf(' [T_SAT]'); end
            if any(abs(u_cmd(2:4)) > 1.5), fprintf(' [τ_SAT]'); end
            fprintf('\n');
        end

        % Logging and Timing
        log_data = update_log(log_data, t, x_curr, x_err, u_cmd);
        t = t + dt_dyn;
        u_prev(1) = u_cmd(1);
        u_prev(2) = u_cmd(2);
        u_prev(3) = u_cmd(3);
        u_prev(4) = u_cmd(4);
        elapsed = toc(loop_start);
        if elapsed < dt_dyn
            pause(dt_dyn - elapsed);
        end
    end
    
    save_log_data(log_data, 'log_p04_lmpc.mat');
    plot_results_template('log_p04_lmpc.mat');

    px4_send_trajectory(client, 0, 0, -5, 0, config);
    pause(10);

    px4_initiate_landing(client, config);
    pause(5);
    px4_disarm_drone(client, config);

    fprintf('LMPC run complete (elapsed: %.1fs)\n', toc(start_time));
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
