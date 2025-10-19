function read_sensors()
%READ_SENSORS Read all sensor data from PX4 via TCP bridge

    % Get configuration
    config = px4_get_config();
    
    fprintf('Reading sensor data from %s:%d\n\n', config.ip_address, config.port);

    try
        % Connect and get telemetry
        client = px4_connect(config.ip_address, config.port);
        telemetry = px4_get_telemetry(client, config);
        
        if isempty(telemetry)
            fprintf('No telemetry data received\n');
            return;
        end
        
        % Display IMU data
        fprintf('--- IMU Data ---\n');
        if isfield(telemetry, 'sensor_combined') && ~isempty(telemetry.sensor_combined)
            imu = telemetry.sensor_combined;
            fprintf('Gyroscope: [%.3f, %.3f, %.3f] rad/s\n', imu.gyro_rad);
            fprintf('Accelerometer: [%.3f, %.3f, %.3f] m/s²\n', imu.accelerometer_m_s2);
        else
            fprintf('No IMU data\n');
        end
        
        % Display GPS data
        fprintf('\n--- GPS Data ---\n');
        if isfield(telemetry, 'gps') && ~isempty(telemetry.gps)
            gps = telemetry.gps;
            fprintf('Position: %.6f°, %.6f°\n', gps.latitude_deg, gps.longitude_deg);
            fprintf('Altitude: %.2f m\n', gps.altitude_msl_m);
            fprintf('Satellites: %d\n', gps.satellites_used);
            fprintf('Accuracy: %.2f m\n', gps.eph);
        else
            fprintf('No GPS data\n');
        end
        
        % Display attitude data
        fprintf('\n--- Attitude Data ---\n');
        if isfield(telemetry, 'attitude') && ~isempty(telemetry.attitude)
            att = telemetry.attitude;
            fprintf('Quaternion: [%.3f, %.3f, %.3f, %.3f]\n', att.q);
            
            % Convert to Euler angles
            [yaw, pitch, roll] = quat2angle([att.q(1), att.q(2), att.q(3), att.q(4)], 'ZYX');
            fprintf('Euler: Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°\n', rad2deg([roll, pitch, yaw]));
        else
            fprintf('No attitude data\n');
        end
        
        % Display position data
        fprintf('\n--- Position Data ---\n');
        if isfield(telemetry, 'local_position') && ~isempty(telemetry.local_position)
            pos = telemetry.local_position;
            fprintf('Position: [%.2f, %.2f, %.2f] m\n', pos.x, pos.y, pos.z);
            fprintf('Velocity: [%.2f, %.2f, %.2f] m/s\n', pos.vx, pos.vy, pos.vz);
            fprintf('Heading: %.1f°\n', rad2deg(pos.heading));
        else
            fprintf('No position data\n');
        end
        
        % Display odometry data
        fprintf('\n--- Odometry Data ---\n');
        if isfield(telemetry, 'odometry') && ~isempty(telemetry.odometry)
            odom = telemetry.odometry;
            fprintf('Position: [%.3f, %.3f, %.3f] m\n', odom.position);
            fprintf('Quaternion: [%.3f, %.3f, %.3f, %.3f]\n', odom.q);
            fprintf('Velocity: [%.3f, %.3f, %.3f] m/s\n', odom.velocity);
            fprintf('Angular Velocity: [%.3f, %.3f, %.3f] rad/s\n', odom.angular_velocity);
        else
            fprintf('No odometry data\n');
        end

        % Display status data
        fprintf('\n--- Status Data ---\n');
        if isfield(telemetry, 'status') && ~isempty(telemetry.status)
            status = telemetry.status;
            arming_states = {'Disarmed', 'Armed'};
            arm_state = arming_states{min(status.arming_state + 1, 2)};
            fprintf('Arming: %s\n', arm_state);
            fprintf('Navigation State: %d\n', status.nav_state);
            fprintf('Failsafe: %s\n', tf_to_string(status.failsafe));
        else
            fprintf('No status data\n');
        end
        
        fprintf('\nSensor data collection complete\n');
        px4_cleanup(client);
        
    catch e
        fprintf('Error: %s\n', e.message);
        if exist('client', 'var')
            px4_cleanup(client);
        end
    end
end

function str = tf_to_string(value)
    if value
        str = 'Active';
    else
        str = 'Inactive';
    end
end