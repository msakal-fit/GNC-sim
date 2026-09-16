% filename: MATLAB/GNC_Algorithms/Control/LQT/traj_line.m

function x_ref = traj_line(t, cfg)
% generate a straihtline as reference trajectory

    % cfg.p0
    % cfg.v
    % cfg.yaw

    p0 = cfg.p0(:);
    v = cfg.v(:);
    yaw = cfg.yaw;

    % position and velocity
    r = p0 + v * t;
    vx = v(1); vy = v(2); vz = v(3);

    % no pitch, roll, keep a constant yaw as zero
    pitch = 0;
    roll = 0;
    q = eul2quat([cfg.yaw, pitch, roll], 'ZYX')'; % [q0; q1; q2; q3]^T

    % no angular velocity
    omega_b = [0; 0; 0];

    x_ref = [r; vx; vy; vz; q; omega_b];
end