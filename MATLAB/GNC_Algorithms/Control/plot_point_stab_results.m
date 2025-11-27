% filename: MATLAB/GNC_Algorithms/Control/plot_point_stab_results.m
%

function plot_point_stab_results(matfile, method_name)

    % Load logged data
    S = load(matfile);
    L = S.log_data;

    t = L.time(:);      % make column
    X = L.state;        % 13 x N
    E = L.error;        % 13 x N
    U = L.control;      % 4 x N
    x_ref = L.x_ref;    % 13 x 1

    close all; % close previous figures

    fig_width  = 800;
    fig_height = 600;

    % 1) States
    fig1 = figure;
    set(fig1, 'Units','pixels', 'Position',[100 100 fig_width fig_height]);

    % Position + reference
    subplot(3,1,1);
    plot(t, X(1,:), 'LineWidth', 1.8); hold on;
    plot(t, X(2,:), 'LineWidth', 1.8);
    plot(t, X(3,:), 'LineWidth', 1.8);
    plot(t, x_ref(1)*ones(size(t)), '--', 'LineWidth', 1.5);
    plot(t, x_ref(2)*ones(size(t)), '--', 'LineWidth', 1.5);
    plot(t, x_ref(3)*ones(size(t)), '--', 'LineWidth', 1.5);
    hold off;
    
    grid on;
    ylabel('pos [m]');
    legend('x','y','z','x_{ref}','y_{ref}','z_{ref}','Location','best');
    title(['States - ', method_name]);

    % Velocity
    subplot(3,1,2);
    plot(t, X(4,:), 'LineWidth', 1.8); hold on;
    plot(t, X(5,:), 'LineWidth', 1.8);
    plot(t, X(6,:), 'LineWidth', 1.8);
    hold off;
    grid on;
    ylabel('vel [m/s]');
    legend('v_x','v_y','v_z','Location','best');

    % Body rates
    subplot(3,1,3);
    plot(t, X(11,:), 'LineWidth', 1.8); hold on;
    plot(t, X(12,:), 'LineWidth', 1.8);
    plot(t, X(13,:), 'LineWidth', 1.8);
    hold off;
    grid on;
    ylabel('\omega [rad/s]');
    xlabel('t [s]');
    legend('p','q','r','Location','best');


    fig2 = figure;
    set(fig2, 'Units','pixels', 'Position',[150 150 fig_width fig_height]);

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

    fig3 = figure;
    set(fig3, 'Units','pixels', 'Position',[200 200 fig_width fig_height]);

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

    fprintf('\n=== Position Error Metrics (%s) ===\n', method_name);
    fprintf('RMSE [ex, ey, ez] = [%.3f, %.3f, %.3f] m\n', ...
            rmse_pos(1), rmse_pos(2), rmse_pos(3));
    fprintf('RMSE of norm(e)       = %.3f m\n', rmse_norm);
    fprintf('Max err [ex, ey, ez] = [%.3f, %.3f, %.3f] m\n', ...
            max_abs_pos_err(1), max_abs_pos_err(2), max_abs_pos_err(3));
    fprintf('Max of norm(e)        = %.3f m\n\n', max_norm);

end