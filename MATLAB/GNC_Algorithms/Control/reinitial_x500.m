config = px4_get_config();
client = px4_connect(config.ip_address, config.port);

px4_initiate_landing(client, config);
pause(5);
px4_disarm_drone(client, config);