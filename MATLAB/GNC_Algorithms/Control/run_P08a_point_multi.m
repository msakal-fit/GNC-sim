% filename: MATLAB/GNC_Algorithms/Control/run_P08a_point_multi.m

function run_P08a_point_multi()

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
            flight_duration = 60;
        case 'attitude'
            dt_dyn = 1/250;
            flight_duration = 10;
        case 'rates'
            dt_dyn = 1/250;
            flight_duration = 15;
    end

    Ts = dt_dyn;

    N_horizon = 25;                       % prediction horizon length
    fprintf('Setting up NMPC with Ts=%.3f s and N=%d\n', Ts, N_horizon);

    % time horizon in seconds
    time_horizon = Ts * N_horizon;
    fprintf('Time horizon: %.3f seconds\n', time_horizon);

    nmpc = p08a_setup_mpc_multi(px4_config, Ts, N_horizon);
    
    % TARGET/REFERENCE STATES
    x_ref = [ ...
        0; 0; -1; ...  % position
        0; 0; 0; ...  % velocity
        1; 0; 0; 0; ...  % quaternion
        0; 0; 0];  % body rates

    % Initial internal state
    x = zeros(13,1);
    x(1:3)   = [0; 0; 0];
    x(7)     = 1.0;
    x(8:10)  = 0.0;
    
    % FEEDFORWARD CONTROL INPUT
    m = px4_config.m; g = px4_config.g;
    U_eq = [m * g; 0; 0; 0];
    
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
    fprintf('Starting the NMPC Point Stab. Multi simulation...\n');
    t = 0;
    
    while t < flight_duration
        loop_start = tic;
        
        % get telemetry data
        telemetry = px4_get_telemetry(client, config);
        
        % get current state vector
        x_curr = state_vec(telemetry);

        x_curr(7:10) = x_curr(7:10) / norm(x_curr(7:10));
        
        % IMPLEMENT YOUR CONTROLLER HERE
        % state error
        x_err = x_curr - x_ref;

        % --- NMPC block ---
        % with current state: x_curr
        % compute the LQR around current state
        % Given x_curr and x_ref, compute optimal control u_nmpc
        [u_nmpc, aux_nmpc] = p08_mpc_step(x_curr, x_ref, nmpc);

        
        % saturate the control inputs
        u_sat = saturate_control(u_nmpc, px4_config);
       
        % Nonlinear dynamics integration
        dynamics_func = @(t, x) drone_nonlinear_dynamics(t, x, u_sat, px4_config);
        x_next = RK4(dynamics_func, x_curr, dt_dyn, t);
        
        % Normalize the predicted quaternion
        x_next(7:10) = x_next(7:10) / norm(x_next(7:10));
        
        thrust_cmd = u_sat(1);
        tau_cmd = u_sat(2:4);
        
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
                    u_sat(1), tau_cmd(1), tau_cmd(2), tau_cmd(3));
            if u_sat(1) ~= u_nmpc(1), fprintf(' [T_SAT]'); end
            if any(abs(u_nmpc(2:4)) > 1.5), fprintf(' [τ_SAT]'); end
            fprintf('\n');
        end

        % Logging and Timing
        log_data = update_log(log_data, t, x_curr, x_err, u_sat);
        i_log = log_data.index - 1;
        log_data.pred(:, i_log) = x_next;

        t = t + dt_dyn;
        elapsed = toc(loop_start);

        % Store CPU time per control step
        i = log_data.index - 1;           % last written sample
        log_data.step_time(i) = elapsed;  % seconds

        if elapsed < dt_dyn
            pause(dt_dyn - elapsed);
        end
    end

    log_data.x_ref = x_ref;
    
    save_log_data(log_data, 'log_p08a_nmpc.mat');
    plot_point_stab_results('log_p08a_nmpc.mat', ' P08a - NMPC Multi-Shooting');
    
    %plot_P06_nmpc_results('log_p08a_nmpc.mat'); % reuse the same function with p06
    % px4_initiate_landing(client, config);
    % pause(5);
    % px4_disarm_drone(client, config);
    reinitial_x500();

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
