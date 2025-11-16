% filename: MATLAB/GNC_Algorithms/Control/LMPC/lmpc_step.m
%
% One LMPC solve:
%   - build QP 
%   - solve with quadprog
%   - return u0 (first input) and some diagnostics

function [u0, info] = lmpc_step(x_curr, A_d, B_d, mpc)

    % Unpack MPC settings
    N     = mpc.N;
    Q     = mpc.Q;
    R     = mpc.R;
    P     = mpc.P;
    x_min = mpc.x_min;
    x_max = mpc.x_max;
    u_min = mpc.u_min;
    u_max = mpc.u_max;

    % Build QP matrices
    [H, f, Aeq, beq, lb, ub, idx] = build_lmpc_qp( ...
        A_d, B_d, Q, R, P, N, x_curr, x_min, x_max, u_min, u_max);

    % quadprog options (no spam)
    opts = optimoptions('quadprog', ...
                        'Display', 'off', ...
                        'Algorithm', 'interior-point-convex');

    % Solve QP
    [z_opt, fval, exitflag, output] = quadprog(H, f, [], [], Aeq, beq, lb, ub, [], opts);

    % Check if QP fails
    nu = size(B_d, 2);
    if exitflag ~= 1
        warning('LMPC: quadprog failed with exitflag = %d. Using zero input.', exitflag);
        u0 = zeros(nu, 1);
    else
        % Extract the first input block: u0*
        u0 = z_opt(idx.inputs(0));
    end

    % Collect some info for debugging / plotting if needed
    info.z_opt    = z_opt;
    info.fval     = fval;
    info.exitflag = exitflag;
    info.output   = output;
end