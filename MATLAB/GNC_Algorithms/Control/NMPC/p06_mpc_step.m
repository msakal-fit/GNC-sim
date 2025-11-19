% filename: MATLAB/GNC_Algorithms/Control/NMPC/p06_mpc_step.m

function [u_applied, aux] = p06_mpc_step(x_curr, x_ref, nmpc)

    %import casadi.*

    x_curr = x_curr(:);
    x_ref  = x_ref(:);

    N    = nmpc.N;
    nu   = nmpc.n_controls;
    Uref = nmpc.U_ref;

    P = [x_curr(:); x_ref(:)];

    % Initial guess for U (always hover)
    U_init_mat = repmat(Uref, 1, N);        % (4 x N)
    U_init     = reshape(U_init_mat, nu*N, 1);  % (4N x 1)
    


    % call NMPC solver
    sol = nmpc.solver(...
        'x0', U_init, ...
        'lbx', nmpc.args.lbx, ...
        'ubx', nmpc.args.ubx, ...
        'lbg', nmpc.args.lbg, ...
        'ubg', nmpc.args.ubg, ...
        'p',  P);

    % extract solution
    U_opt = full(sol.x);                     % (4N x 1)
    U_opt_mat = reshape(U_opt, nu, N);       % (4 x N)

    % apply first control input
    u_applied = U_opt_mat(:,1);

    % pack into
    aux = struct;
    aux.U_opt        = U_opt_mat;           % full optimal sequence
    aux.solver_stats = nmpc.solver.stats(); % IPOPT stats

end

