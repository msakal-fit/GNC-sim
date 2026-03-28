% filename: MATLAB/GNC_Algorithms/Control/LQT/plot_P02_lqt_results.m

function plot_P02_lqt_results(matfile)
    S = load(matfile); L = S.log_data;
    t = L.time(:); X = L.state; E = L.error; U = L.control; Xr = L.ref;

    % position vs ref
    figure; tiledlayout(3,1,'Padding','compact');
    lbl = {'x','y','z'};
    for k=1:3
        nexttile; plot(t, X(k,:), t, Xr(k,:),'--'); grid on;
        ylabel(sprintf('%s [m]', lbl{k})); legend actual ref;
    end
    xlabel('t [s]'); sgtitle('Position tracking');

    % velocity vs ref
    figure; tiledlayout(3,1,'Padding','compact');
    lbl = {'v_x','v_y','v_z'};
    for k=1:3
        nexttile; plot(t, X(3+k,:), t, Xr(3+k,:),'--'); grid on;
        ylabel(sprintf('%s [m/s]', lbl{k})); legend actual ref;
    end
    xlabel('t [s]'); sgtitle('Velocity tracking');

    % yaw (from quaternion) vs ref yaw
    psi_act = quatYawSeries(X(7:10,:));
    psi_ref = quatYawSeries(Xr(7:10,:));
    figure; plot(t, psi_act, t, psi_ref,'--'); grid on;
    xlabel('t [s]'); ylabel('\psi [rad]'); legend actual ref; title('Yaw tracking');

    % errors (position + rates)
    figure; tiledlayout(2,1,'Padding','compact');
    nexttile; plot(t, E(1:3,:)); grid on; ylabel('pos err [m]'); legend e_x e_y e_z;
    nexttile; plot(t, E(11:13,:)); grid on; ylabel('rate err [rad/s]'); legend e_p e_q e_r; xlabel('t [s]');
    sgtitle('Errors');

    % inputs
    figure; plot(t, U); grid on; xlabel('t [s]');
    ylabel('u = [T,\tau_x,\tau_y,\tau_z]'); legend T \tau_x \tau_y \tau_z; title('Control input');
end

function psi = quatYawSeries(Q)
    % Q: 4xN, [q0;q1;q2;q3]
    psi = zeros(1,size(Q,2));
    for i=1:size(Q,2)
        e = quat2eul(Q(:,i)', 'ZYX'); % [yaw pitch roll]
        psi(i) = e(1);
    end
end