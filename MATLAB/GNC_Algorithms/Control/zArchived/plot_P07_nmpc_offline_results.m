% filename: MATLAB/GNC_Algorithms/Control/NMPC/plot_P07_nmpc_offline_results.m
%
% Plot position vs reference, tracking error, and control inputs
% for the offline NMPC simulation.

function plot_P07_nmpc_offline_results(t_state, t_input, X_hist, X_ref_hist, U_hist)

    % X_hist:     13 x (N_steps+1)
    % X_ref_hist: 13 x  N_steps
    % U_hist:      4 x  N_steps

    % align sizes
    x = X_hist;
    x_ref = [X_ref_hist, X_ref_hist(:,end)];   % extend last ref to match length

    % Positions
    px      = x(1,:);
    py      = x(2,:);
    pz      = x(3,:);
    px_ref  = x_ref(1,:);
    py_ref  = x_ref(2,:);
    pz_ref  = x_ref(3,:);

    % Control inputs
    T_hist   = U_hist(1,:);
    tau_x    = U_hist(2,:);
    tau_y    = U_hist(3,:);
    tau_z    = U_hist(4,:);

    % ---------------------------------------------------------------------
    % 1) XY trajectory (circle tracking)
    % ---------------------------------------------------------------------
    figure;
    plot(px_ref, py_ref, 'r--', 'LineWidth', 1.5); hold on;
    plot(px,     py,     'b',   'LineWidth', 1.5);
    axis equal;
    grid on;
    xlabel('x [m]');
    ylabel('y [m]');
    legend('Reference','Actual','Location','Best');
    title('NMPC Trajectory Tracking (XY plane)');

    % ---------------------------------------------------------------------
    % 2) Position vs reference (x,y,z)
    % ---------------------------------------------------------------------
    figure;
    subplot(3,1,1);
    plot(t_state, px_ref, 'r--', 'LineWidth', 1.2); hold on;
    plot(t_state, px,     'b',   'LineWidth', 1.2);
    grid on; ylabel('x [m]');
    legend('reference','actual','Location','Best');
    title('Position Tracking');

    subplot(3,1,2);
    plot(t_state, py_ref, 'r--', 'LineWidth', 1.2); hold on;
    plot(t_state, py,     'b',   'LineWidth', 1.2);
    grid on; ylabel('y [m]');

    subplot(3,1,3);
    plot(t_state, pz_ref, 'r--', 'LineWidth', 1.2); hold on;
    plot(t_state, pz,     'b',   'LineWidth', 1.2);
    grid on; ylabel('z [m]');
    xlabel('time [s]');

    % ---------------------------------------------------------------------
    % 3) Position error
    % ---------------------------------------------------------------------
    ex = px - px_ref;
    ey = py - py_ref;
    ez = pz - pz_ref;
    e_norm = sqrt(ex.^2 + ey.^2 + ez.^2);

    figure;
    subplot(4,1,1);
    plot(t_state, ex, 'LineWidth', 1.2); grid on;
    ylabel('e_x [m]');
    title('Position Tracking Error');

    subplot(4,1,2);
    plot(t_state, ey, 'LineWidth', 1.2); grid on;
    ylabel('e_y [m]');

    subplot(4,1,3);
    plot(t_state, ez, 'LineWidth', 1.2); grid on;
    ylabel('e_z [m]');

    subplot(4,1,4);
    plot(t_state, e_norm, 'LineWidth', 1.2); grid on;
    ylabel('abs|e| [m]');
    xlabel('time [s]');

    % ---------------------------------------------------------------------
    % 4) Control inputs
    % ---------------------------------------------------------------------
    figure;
    subplot(4,1,1);
    plot(t_input, T_hist, 'LineWidth', 1.2); grid on;
    ylabel('T [N]');
    title('Control Inputs');

    subplot(4,1,2);
    plot(t_input, tau_x, 'LineWidth', 1.2); grid on;
    ylabel('\tau_x [N·m]');

    subplot(4,1,3);
    plot(t_input, tau_y, 'LineWidth', 1.2); grid on;
    ylabel('\tau_y [N·m]');

    subplot(4,1,4);
    plot(t_input, tau_z, 'LineWidth', 1.2); grid on;
    ylabel('\tau_z [N·m]');
    xlabel('time [s]');

end
