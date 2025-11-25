% filename: MATLAB/GNC_Algorithms/Control/NMPC/p07_mpc_step.m

function [u_applied, aux] = p07_mpc_step(x_curr, Xref_hor, u_prev, nmpc)

    x_curr = x_curr(:);
    u_prev = u_prev(:);

    N    = nmpc.N;
    nx   = nmpc.n_states;
    nu   = nmpc.n_controls;
    Uref = nmpc.U_ref;

    % build parameter vector P = [x0; x_ref^(1); ...; x_ref^(N); u_prev]
    P = zeros(nx*(N+1) + nu, 1);
    P(1:nx) = x_curr;

    for k = 1:N
        idx = nx*k + (1:nx);
        P(idx) = Xref_hor(:,k);
    end

    P(nx*(N+1)+1 : end) = u_prev;

    % Initial guess for U (use hover)
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

