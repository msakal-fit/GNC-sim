% filename: MATLAB/GNC_Algorithms/Control/plot_P01_lqr_results.m

function plot_P01_lqr_results(matfile)
    S = load(matfile);
    L = S.log_data;

    t = L.time(:);
    X = L.state;
    E = L.error;
    U = L.control;

    figure; tiledlayout(3,1,'Padding','compact');
    nexttile; plot(t, X(1:3,:)); grid on; ylabel('pos [m]'); legend x y z;
    nexttile; plot(t, X(4:6,:)); grid on; ylabel('vel [m/s]'); legend vx vy vz;
    nexttile; plot(t, X(11:13,:)); grid on; ylabel('\omega [rad/s]'); xlabel('t [s]'); legend p q r;
    sgtitle('States');

    figure; tiledlayout(3,1,'Padding','compact');
    nexttile; plot(t, E(1:3,:)); grid on; ylabel('pos err [m]'); legend ex ey ez;
    nexttile; plot(t, E(7:10,:)); grid on; ylabel('quat err'); legend e_{q0} e_{q1} e_{q2} e_{q3};
    nexttile; plot(t, E(11:13,:)); grid on; ylabel('rate err [rad/s]'); xlabel('t [s]'); legend ep eq er;
    sgtitle('Errors');

    figure; plot(t, U); grid on; xlabel('t [s]'); ylabel('u = [T,\tau_x,\tau_y,\tau_z]');
    legend T \tau_x \tau_y \tau_z; title('Control Input');
end
