function test_simulator()

    % Establish connection between MATLAB and simulator
    config = px4_get_config();
    client = px4_connect(config.ip_address, config.port);

     % modify takeoff altitude, if needed.
    config.takeoff_altitude = -5.0;

    % arm the drone for manual control
    px4_enter_offboard_mode(client, config);
    pause(2);

    px4_arm_drone(client, config);
    pause(2);
    
    disp('Initiate take-off')
    px4_takeoff(client, config);
    pause(15);
    
    % Verify takeoff 
    telemetry = px4_get_telemetry(client, config);
    current_alt = -telemetry.odometry.position(3);
    
    if current_alt < 1.5
        error('Takeoff failed - altitude: %.1fm', current_alt);
    end
    
    fprintf('Takeoff complete at %.1fm\n', current_alt);
    pause(10);

    disp('Initiate Landing...')
    px4_initiate_landing(client, config);
    pause(5);
    px4_disarm_drone(client, config);
    disp('Landed and disarmed')

end