config = px4_get_config();
client = px4_connect(config.ip_address, config.port);

px4_enter_offboard_mode(client, config);
pause(2);

px4_arm_drone(client, config);
pause(2);

% MODIFY TAKEOFF ALTITUDE, IF NEEDED.
config.takeoff_altitude = -5.0;

px4_takeoff(client, config);
pause(10);

px4_send_trajectory(client, 0, 0, -5, 0, config);

pause(10);

px4_initiate_landing(client, config);
pause(5);
px4_disarm_drone(client, config);