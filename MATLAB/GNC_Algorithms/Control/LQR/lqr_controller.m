% filename: MATLAB/GNC_Algorithms/Control/LQR/lqr_controller.m

function [u, K, aux] = lqr_controller(x_curr, x_ref, params, Q, R)
    % LQR Controller

    % Inputs:
    %   x_curr: current state vector
    %   x_target: target state vector
    %   params: structure containing necessary parameters

    % Outputs:
    %   u: control input
    %   K: LQR gain matrix
    %   aux: struct with diagnostic information (A, B, U_eq, err)

    m = params.m;
    g = params.g;

    % --- hover input
    U_eq = [m * g; 0; 0; 0];

    % --- normalize quaternion
    x_curr(7:10) = x_curr(7:10) / norm(x_curr(7:10));
    x_ref(7:10) = x_ref(7:10) / norm(x_ref(7:10));

    % --- build linearized system matrices at current state
    [A, B] = drone_linear_dynamics(x_curr, U_eq, params);

    % --- compute LQR gain
    K = lqr(A, B, Q, R);

    % --- error vector
    x_err = x_curr - x_ref;

    % --- compute control law
    u = U_eq - K * x_err;

    % return auxiliary information
    aux = struct('A', A, 'B', B, 'U_eq', U_eq, 'x_err', x_err);
end
