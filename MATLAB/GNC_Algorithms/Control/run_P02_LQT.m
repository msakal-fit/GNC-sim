% filename: MATLAB/GNC_Algorithms/Control/run_P02_LQT.m

function run_P02_LQT()

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
    
    % TARGET/REFERENCE STATES

    % Trajectory configuration
    % refcfg.R = 4.0;  % meters
    % refcfg.V = 0.5;  % meters/second
    % refcfg.center = [0; 0];  % circle center in XY plane
    % refcfg.z0 = -2.0;  % initial altitude

    refcfg.z0 = config.takeoff_altitude;  % constant altitude for line traj

    % Trajectory configuration for straight line
    refcfg.p0  = [0; 0; refcfg.z0];   % start at (0,0,z0)
    refcfg.v   = [0.1; 0; 0];         % 0.5 m/s along +x
    refcfg.yaw = 0;                   % face +x

    % x_target = [ ...
    %     0; 0; 0; ...  % position
    %     0; 0; 0; ...  % velocity
    %     1; 0; 0; 0; ...  % quaternion
    %     0; 0; 0];  % body rates

    % WEIGHTS 
    Q_pos = diag([  50,  5, 5 ]);
    Q_vel = diag([  12,  2,  2 ]);
    Q_q   = diag([ 20, 20, 20, 20 ]);
    Q_omega  = diag([  2,  2,  2 ]);
    Q = blkdiag(Q_pos, Q_vel, Q_q, Q_omega);

    R = diag([ 2,  1,  1,  1 ]);

    % print weights for logging purposes
    disp('State weight matrix Q as diag:');
    disp(Q_pos);
    disp(Q_vel);
    disp(Q_q);
    disp(Q_omega);
    disp('Input weight matrix R as diag:');
    disp(R);
    
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

    config.takeoff_altitude = refcfg.z0; % Set takeoff altitude to match trajectory
    
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
    fprintf('Starting the LQT simulation...\n');
    t = 0;
    
    x_eq       = zeros(13,1);
    x_eq(7)    = 1;        % q = [1 0 0 0]^T
    U_eq = [m*g; 0; 0; 0];
    [A_eq, B_eq] = drone_linear_dynamics(x_eq, U_eq, px4_config);
    K = lqr(A_eq, B_eq, Q, R);
    
    while t < flight_duration
        loop_start = tic;
        
        % get telemetry data
        telemetry = px4_get_telemetry(client, config);
        
        % IMPLEMENT YOUR CONTROLLER HERE
        % --- LQR block ---
        % get current state vector
        x_curr = state_vec(telemetry);
        x_curr(7:10) = x_curr(7:10) / norm(x_curr(7:10));
        % with current state: x_curr
        % compute the LQR around current state
        % [u_lqr, K, aux] = lqr_controller(x_curr, xref, px4_config, Q, R);
        
        % x_err = aux.x_err;
        % xref = traj_circle(t, refcfg);
        xref = traj_line(t, refcfg);

        x_err = x_curr - xref;
        u_lqr = U_eq - K * x_err;
        
        % saturate the control inputs
        u_sat = saturate_control(u_lqr, px4_config);
       
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
                %yaw_ref = atan2(xref(5), xref(4));
                %px4_send_trajectory(client, xref(1), xref(2), xref(3), yaw_ref, config);
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
            if u_sat(1) ~= u_lqr(1), fprintf(' [T_SAT]'); end
            if any(abs(u_lqr(2:4)) > 1.5), fprintf(' [τ_SAT]'); end
            fprintf('\n');
        end

        % Logging and Timing
        % --- Log ref ---
        log_data = update_log(log_data, t, x_curr, x_curr - xref, u_sat);  % or u_sat if you keep it
        if ~isfield(log_data,'ref'), log_data.ref = []; end
        log_data.ref(:, end+1) = xref;

        
        t = t + dt_dyn;
        elapsed = toc(loop_start);

        % Store CPU time per control step
        i = log_data.index - 1;           % last written sample
        log_data.step_time(i) = elapsed;  % seconds

        if elapsed < dt_dyn
            pause(dt_dyn - elapsed);
        end
    end
    
    save_log_data(log_data, 'log_p02_lqt.mat');
    %plot_P02_lqt_results('log_p02_lqt.mat');
    plot_tracking_results('log_p02_lqt.mat', 'P02 - LQT');

    reinitial_x500("tracking");

    % px4_send_trajectory(client, 0, 0, -5, 0, config);
    % pause(10);

    % px4_initiate_landing(client, config);
    % pause(5);
    % px4_disarm_drone(client, config);
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
