function x_ref = traj_circle(t, cfg)
% x_ref: [r(3); v(3); q(4); omega_b(3)]

    R = cfg.R;
    V = cfg.V;
    cxy = cfg.center(:);
    z0 = cfg.z0;

    w = V / R;  % angular velocity
    cx = cxy(1); cy = cxy(2);

    % positon (circle in XY), altitude z0
    x = cx + R * cos(w * t);
    y = cy + R * sin(w * t);
    z = z0;

    % velocity
    vx = -R * w * sin(w * t);
    vy = R * w * cos(w * t);
    vz = 0;

    % acceleration
    ax = -R * w^2 * cos(w * t);
    ay = -R * w^2 * sin(w * t);
    az = 0;

    % yaw angle (tangent to the circle)
    yaw = atan2(vy, vx);
    pitch = 0;
    roll = 0;

    yaw_dot = (vx * ay - vy * ax) / (vx^2 + vy^2);

    % quaternion (from roll, pitch, yaw)
    q = eul2quat([yaw, pitch, roll], 'ZYX')';

    % body rates (from yaw_dot)
    omega_b = [0; 0; yaw_dot];

    % reference state
    x_ref = [x; y; z; vx; vy; vz; q; omega_b];
end
