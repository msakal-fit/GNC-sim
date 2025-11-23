% filename: MATLAB/GNC_Algorithms/Control/LQG/kalman_filter_update.m

function [x_pred, P_pred, aux] = kalman_filter_update( ...
    x_hat_prev, P_prev, u_prev, params, Q_k, R_k, dt)

    % linear kalman filter for quadcopter with 13-states


    % Inputs


    % Outputs


    nx = 13; % number of states


    % build linearlized continuous-time model at the current estimate

    m = params.m;
    g = params.g;
    U_eq = [m*g; 0; 0; 0]; % equilibrium input

    [A_c, B_c] = drone_linear_dynamics(x_hat_prev, U_eq, params);


    % discretize using forward Euler
    F = eye(nx) + A_c*dt; % 13x13
    G = B_c*dt;            % 13x4


    % prediction step
    x_pred = F*x_hat_prev + G*u_prev; % state nx x 1
    P_pred = F*P_prev*F' + Q_k;       % covariance nx x nx

    % measurement model
    % use simple model where y = x + v, where v is measurement noise
    % assume that we can measure all states directly
    H = eye(nx); % 

    % update the Kalman gain
    S = H*P_pred*H' + R_k;          % innovation covariance, nx x nx
    K = P_pred*H' / S;              % Kalman gain, nx x nx

    % return the K in aux to be used in the main loop

    aux = struct();
    aux.K = K;              % Kalman gain
    aux.F = F;              % state transition matrix
    aux.G = G;              % control input matrix
    aux.x_pred = x_pred;    % predicted state
    aux.P_pred = P_pred;    % predicted covariance

    % % measurement update step
    % x_hat_plus = x_pred;
    % P_plus = P_pred;

end