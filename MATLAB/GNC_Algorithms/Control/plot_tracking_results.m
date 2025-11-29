% filename: MATLAB/GNC_Algorithms/Control/plot_tracking_results.m
%

function plot_tracking_results(matfile, method_name)

    % Load logged data
    S = load(matfile);
    L = S.log_data;

    t = L.time(:);      % make column
    X = L.state;        % 13 x N
    E = L.error;        % 13 x N
    U = L.control;      % 4 x N
    x_ref = L.ref;    % 13 x N

    close all; % close previous figures

    fig_width  = 800;
    fig_height = 600;

    % states
    fig_states = figure;
    set(fig_states, 'Units','pixels', 'Position',[100 100 fig_width fig_height]);

    % Position + reference
    subplot(3,1,1);
    plot(t, X(1,:), 'LineWidth', 1.8); hold on;
    plot(t, X(2,:), 'LineWidth', 1.8);
    plot(t, X(3,:), 'LineWidth', 1.8);
    plot(t, x_ref(1,:)', '--', 'LineWidth', 1.5);
    plot(t, x_ref(2,:)', '--', 'LineWidth', 1.5);
    plot(t, x_ref(3,:)', '--', 'LineWidth', 1.5);
    hold off;
    
    grid on;
    ylabel('pos [m]');
    legend('x','y','z','x_{ref}','y_{ref}','z_{ref}','Location','best');
    title(['States (Tracking) - ', method_name]);

    % Velocity
    subplot(3,1,2);
    plot(t, X(4,:), 'LineWidth', 1.8); hold on;
    plot(t, X(5,:), 'LineWidth', 1.8);
    plot(t, X(6,:), 'LineWidth', 1.8);
    plot(t, x_ref(4,:)', '--', 'LineWidth', 1.5);
    plot(t, x_ref(5,:)', '--', 'LineWidth', 1.5);
    plot(t, x_ref(6,:)', '--', 'LineWidth', 1.5);
    hold off;
    grid on;
    ylabel('vel [m/s]');
    legend('v_x','v_y','v_z','v_{x,ref}','v_{y,ref}','v_{z,ref}','Location','best');

    % Body rates
    subplot(3,1,3);
    plot(t, X(11,:), 'LineWidth', 1.8); hold on;
    plot(t, X(12,:), 'LineWidth', 1.8);
    plot(t, X(13,:), 'LineWidth', 1.8);
    plot(t, x_ref(11,:)', '--', 'LineWidth', 1.5);
    plot(t, x_ref(12,:)', '--', 'LineWidth', 1.5);
    plot(t, x_ref(13,:)', '--', 'LineWidth', 1.5);
    hold off;
    grid on;
    ylabel('\omega [rad/s]');
    xlabel('t [s]');
    legend('p','q','r','p_{ref}','q_{ref}','r_{ref}','Location','best');

    % attitude

    % Extract quaternion components from state
    q0 = X(7,:);
    q1 = X(8,:);
    q2 = X(9,:);
    q3 = X(10,:);

    q0_ref = x_ref(7,:);
    q1_ref = x_ref(8,:);
    q2_ref = x_ref(9,:);
    q3_ref = x_ref(10,:);

    N = length(t);
    eul_act = zeros(N,3);   % [yaw, pitch, roll] for actual attitude
    eul_ref = zeros(N,3);   % [yaw, pitch, roll] for reference attitude

    for k = 1:N
        qa = [q0(k)    q1(k)    q2(k)    q3(k)];
        qa = qa / norm(qa);                 % safety normalization
        eul_act(k,:) = quat2eul(qa, 'ZYX');     % [yaw, pitch, roll]


        qr = [q0_ref(k) q1_ref(k) q2_ref(k) q3_ref(k)];
        qr = qr / norm(qr);                 % safety normalization
        eul_ref(k,:) = quat2eul(qr, 'ZYX');     % [yaw, pitch, roll]
    end

    roll_act      = eul_act(:,3);
    pitch_act     = eul_act(:,2);
    yaw_act       = eul_act(:,1);

    roll_ref  = eul_ref(:,3);
    pitch_ref = eul_ref(:,2);
    yaw_ref   = eul_ref(:,1);

    fig_att = figure;
    set(fig_att, 'Units','pixels', 'Position',[130 130 fig_width fig_height]);

    % Quaternion components
    subplot(2,1,1);
    plot(t, q0, 'LineWidth', 1.8); hold on;
    plot(t, q1, 'LineWidth', 1.8);
    plot(t, q2, 'LineWidth', 1.8);
    plot(t, q3, 'LineWidth', 1.8);
    hold off;
    grid on;
    ylabel('q');
    legend('q_0','q_1','q_2','q_3','Location','best');
    title(['Attitude (Quaternion + Euler) - ', method_name]);

    % Euler angles in degrees (actual vs reference)
    subplot(2,1,2);
    plot(t, rad2deg(roll_act),      'LineWidth', 1.8); hold on;
    plot(t, rad2deg(pitch_act),     'LineWidth', 1.8);
    plot(t, rad2deg(yaw_act),       'LineWidth', 1.8);
    plot(t, rad2deg(roll_ref),  '--', 'LineWidth', 1.5);
    plot(t, rad2deg(pitch_ref), '--', 'LineWidth', 1.5);
    plot(t, rad2deg(yaw_ref),   '--', 'LineWidth', 1.5);
    hold off;
    grid on;
    xlabel('t [s]');
    ylabel('angle [deg]');
    legend('roll (act)','pitch (act)','yaw (act)','roll (ref)','pitch (ref)','yaw (ref)','Location','best');

    fig_err = figure;
    set(fig_err, 'Units','pixels', 'Position',[150 150 fig_width fig_height]);

    % Position error
    subplot(3,1,1);
    plot(t, E(1,:), 'LineWidth', 1.8); hold on;
    plot(t, E(2,:), 'LineWidth', 1.8);
    plot(t, E(3,:), 'LineWidth', 1.8);
    hold off;
    grid on;
    ylabel('pos err [m]');
    legend('e_x','e_y','e_z','Location','best');
    title(['Errors - ', method_name]);

    % Quaternion error
    subplot(3,1,2);
    plot(t, E(7,:), 'LineWidth', 1.8); hold on;
    plot(t, E(8,:), 'LineWidth', 1.8);
    plot(t, E(9,:), 'LineWidth', 1.8);
    plot(t, E(10,:), 'LineWidth', 1.8);
    hold off;
    grid on;
    ylabel('quat err');
    legend('e_{q0}','e_{q1}','e_{q2}','e_{q3}','Location','best');

    % Rate error
    subplot(3,1,3);
    plot(t, E(11,:), 'LineWidth', 1.8); hold on;
    plot(t, E(12,:), 'LineWidth', 1.8);
    plot(t, E(13,:), 'LineWidth', 1.8);
    hold off;
    grid on;
    ylabel('rate err [rad/s]');
    xlabel('t [s]');
    legend('e_p','e_q','e_r','Location','best');


    % Control inputs
    fig_ctrl = figure;
    set(fig_ctrl, 'Units','pixels', 'Position',[200 200 fig_width fig_height]);

    % Thrust
    subplot(2,1,1);
    plot(t, U(1,:), 'LineWidth', 1.8);
    grid on;
    ylabel('T [N]');
    title(['Control Input - ', method_name]);

    % Torques
    subplot(2,1,2);
    plot(t, U(2,:), 'LineWidth', 1.8); hold on;
    plot(t, U(3,:), 'LineWidth', 1.8);
    plot(t, U(4,:), 'LineWidth', 1.8);
    hold off;
    grid on;
    xlabel('t [s]');
    ylabel('\tau [Nm]');
    legend('\tau_x','\tau_y','\tau_z','Location','best');

    % 3D trajectory: actual vs reference
    fig_traj3d = figure;
    set(fig_traj3d, 'Units','pixels', 'Position',[230 230 fig_width fig_height]);

    plot3(X(1,:),    X(2,:),    X(3,:),    'LineWidth', 1.8); hold on;
    plot3(x_ref(1,:),x_ref(2,:),x_ref(3,:),'--', 'LineWidth', 1.8);
    hold off;
    grid on; axis equal;
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    legend('actual','reference','Location','best');
    title(['3D Trajectory - ', method_name]);
    view(3);

    % top view (XY)
    fig_traj2d = figure;
    set(fig_traj2d, 'Units','pixels', 'Position',[260 260 fig_width fig_height]);

    plot(X(1,:),    X(2,:),    'LineWidth', 1.8); hold on;
    plot(x_ref(1,:),x_ref(2,:),'--', 'LineWidth', 1.8);
    hold off;
    grid on; axis equal;
    xlabel('x [m]');
    ylabel('y [m]');
    legend('actual','reference','Location','best');
    title(['XY Trajectory (Top View) - ', method_name]);

    % side view (XZ)
    fig_traj_xz = figure;
    set(fig_traj_xz, 'Units','pixels', 'Position',[290 290 fig_width fig_height]);
    plot(X(1,:),    X(3,:),    'LineWidth', 1.8); hold on;
    plot(x_ref(1,:),x_ref(3,:),'--', 'LineWidth', 1.8);
    hold off;
    grid on; axis equal;
    xlabel('x [m]');
    ylabel('z [m]');
    legend('actual','reference','Location','best');
    title(['X-Z Trajectory (Side View) - ', method_name]);

    %Position error metrics (RMSE + max)
    pos_err = E(1:3,:);                        % 3 x N
    err_norm = sqrt(sum(pos_err.^2, 1));

    % RMSE per axis
    rmse_pos = sqrt(mean(pos_err.^2, 2));      % 3 x 1
    % RMSE of the norm
    rmse_norm = sqrt(mean(err_norm.^2));


    % Max absolute error per axis
    max_abs_pos_err = max(abs(pos_err), [], 2);  % 3 x 1
    % Max norm
    max_norm = max(err_norm);                    % scalar

    % u_norm is the Euclidean norm of the control vector at each step
    u_norm      = sqrt(sum(U.^2, 1));              % 1 x N
    avg_u_norm  = mean(u_norm);
    max_u_norm  = max(u_norm);

    % Average thrust and average torque norm
    avg_T          = mean(U(1,:));
    max_T         = max(U(1,:));
    tau_norm       = sqrt(sum(U(2:4,:).^2, 1));    % 1 x N
    avg_tau_norm   = mean(tau_norm);
    max_tau_norm   = max(tau_norm);

    % --- CPU time per step (if available) ---
    has_cpu = isfield(L, 'step_time');
    if has_cpu
        dt_step   = L.step_time(:);
        avg_dt    = mean(dt_step);
        max_dt    = max(dt_step);
    end

    % --- Print metrics to command window ---
    fprintf('\n=== Metrics (%s) ===\n', method_name);
    fprintf('RMSE [ex, ey, ez] = [%.3f, %.3f, %.3f] m\n', ...
            rmse_pos(1), rmse_pos(2), rmse_pos(3));
    fprintf('RMSE of norm(e)       = %.3f m\n', rmse_norm);
    fprintf('Max err [ex, ey, ez] = [%.3f, %.3f, %.3f] m\n', ...
            max_abs_pos_err(1), max_abs_pos_err(2), max_abs_pos_err(3));
    fprintf('Max of norm(e)        = %.3f m\n\n', max_norm);

    fprintf('Average control effort  mean of (u)   = %.3f\n', avg_u_norm);
    fprintf('Max control effort      max(u)    = %.3f\n', max_u_norm);
    fprintf('Average thrust          mean(T)       = %.3f N\n', avg_T);
    fprintf('Max thrust              max(T)        = %.3f N\n', max_T);
    fprintf('Average torque norm     mean(tau) = %.3f Nm\n', avg_tau_norm);
    fprintf('Max torque norm         max(tau)  = %.3f Nm\n', max_tau_norm);

    if has_cpu
        fprintf('Average CPU time per step  = %.4f s\n', avg_dt);
        fprintf('Max CPU time per step      = %.4f s\n', max_dt);
    else
        fprintf('CPU time per step          = (not logged in this run)\n');
    end

    fprintf('======================================\n\n');

end