function reinitial_x500(flight_mode)
    % Reinitializes the X500 drone to a known state.
    config = px4_get_config();
    client = px4_connect(config.ip_address, config.port);

    px4_enter_offboard_mode(client, config);
    pause(2);

    px4_arm_drone(client, config);
    pause(2);

    % MODIFY TAKEOFF ALTITUDE, IF NEEDED.
    config.takeoff_altitude = -2.0;

    px4_takeoff(client, config);
    pause(7);

    % CONTROLLER SELECTION
    CONTROL_MODE = 'position';

    % switch to control mode
    px4_switch_control_mode(client, CONTROL_MODE, config);
    pause(2)

    px4_send_trajectory(client, 0, 0, -2, 0, config); % hover at (0, 0, -2), yaw = 0
    
    if flight_mode == "point"
        pause(10);
    elseif flight_mode == "tracking"
        pause(60);
    end

    px4_initiate_landing(client, config);
    pause(5);
    px4_disarm_drone(client, config);
end