% filename: MATLAB/GNC_Algorithms/Control/NMPC/plot_P07_nmpc_results.m
%
% Plot results for Problem 7 NMPC trajectory tracking (PX4 in the loop).
% Requires a MAT-file with 'log_data' saved by save_log_data().

function plot_P07_nmpc_results(matfile)
    S = load(matfile);
    L = S.log_data;

    t  = L.time(:);           % Nx1
    X  = L.state;             % 13xN, actual
    E  = L.error;             % 13xN
    U  = L.control;           % 4xN
    Xr = L.ref;               % 13xN, reference from traj_circle
    Xp = L.pred;              % 13xN, predicted state from NMPC (x_next)

    %% 1) XY trajectory: ref vs actual vs predicted
    figure;
    hold on; grid on; axis equal;
    plot(Xr(1,:), Xr(2,:), 'r--', 'LineWidth', 1.5);  % reference circle
    plot(X(1,:),  X(2,:),  'b',   'LineWidth', 1.5);  % actual PX4
    plot(Xp(1,:), Xp(2,:), 'g:',  'LineWidth', 1.5);  % NMPC prediction
    xlabel('x [m]');
    ylabel('y [m]');
    legend('ref','actual','predicted','Location','Best');
    title('XY Trajectory: reference vs actual vs NMPC predicted');

    %% 2) Position vs reference (+ predicted)
    figure; tiledlayout(3,1,'Padding','compact');
    lbl = {'x','y','z'};
    for k = 1:3
        nexttile; hold on; grid on;
        plot(t, Xr(k,:), 'r--', 'LineWidth', 1.2);  % ref
        plot(t, X(k,:),  'b',   'LineWidth', 1.4);  % actual
        plot(t, Xp(k,:), 'g:',  'LineWidth', 1.0);  % pred
        ylabel(sprintf('%s [m]', lbl{k}));
        legend('ref','actual','pred','Location','Best');
    end
    xlabel('t [s]');
    sgtitle('Position tracking (P07 NMPC)');

    %% 3) Velocity vs reference (vx, vy, vz)
    figure; tiledlayout(3,1,'Padding','compact');
    lbl = {'v_x','v_y','v_z'};
    for k = 1:3
        kk = 3 + k; % vx,vy,vz are states 4:6
        nexttile; hold on; grid on;
        plot(t, Xr(kk,:), 'r--', 'LineWidth', 1.2);  % ref vel
        plot(t, X(kk,:),  'b',   'LineWidth', 1.4);  % actual vel
        ylabel(sprintf('%s [m/s]', lbl{k}));
        legend('ref','actual','Location','Best');
    end
    xlabel('t [s]');
    sgtitle('Velocity tracking');

    %% 4) Yaw (from quaternion) vs reference yaw
    psi_act = quatYawSeries(X(7:10,:));
    psi_ref = quatYawSeries(Xr(7:10,:));

    figure; hold on; grid on;
    plot(t, psi_ref,'r--','LineWidth',1.2);
    plot(t, psi_act,'b',  'LineWidth',1.4);
    xlabel('t [s]');
    ylabel('\psi [rad]');
    legend('ref','actual','Location','Best');
    title('Yaw tracking');

    %% 5) Errors (position + rates)
    figure; tiledlayout(2,1,'Padding','compact');
    % position errors
    nexttile; hold on; grid on;
    plot(t, E(1,:), 'LineWidth', 1.2);
    plot(t, E(2,:), 'LineWidth', 1.2);
    plot(t, E(3,:), 'LineWidth', 1.2);
    ylabel('pos err [m]');
    legend('e_x','e_y','e_z','Location','Best');

    % body-rate errors
    nexttile; hold on; grid on;
    plot(t, E(11,:), 'LineWidth', 1.2);
    plot(t, E(12,:), 'LineWidth', 1.2);
    plot(t, E(13,:), 'LineWidth', 1.2);
    ylabel('rate err [rad/s]');
    legend('e_p','e_q','e_r','Location','Best');
    xlabel('t [s]');
    sgtitle('Errors (P07 NMPC + PX4)');

    %% 6) Control inputs (from NMPC)
    figure; hold on; grid on;
    plot(t, U(1,:), 'LineWidth',1.2);
    plot(t, U(2,:), 'LineWidth',1.2);
    plot(t, U(3,:), 'LineWidth',1.2);
    plot(t, U(4,:), 'LineWidth',1.2);
    xlabel('t [s]');
    ylabel('u = [T,\tau_x,\tau_y,\tau_z]');
    legend('T','\tau_x','\tau_y','\tau_z','Location','Best');
    title('Control inputs (NMPC)');
end

function psi = quatYawSeries(Q)
    % Q: 4xN, [q0;q1;q2;q3]
    psi = zeros(1,size(Q,2));
    for i=1:size(Q,2)
        e = quat2eul(Q(:,i)', 'ZYX'); % [yaw pitch roll]
        psi(i) = e(1);                % yaw
    end
end
