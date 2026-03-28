% filename: MATLAB/GNC_Algorithms/Control/NMPC/p08_mpc_step.m

% NMPC step with multiple shooting

function [u_applied, aux] = p08_mpc_step(x_curr, x_ref, nmpc)

    x_curr = x_curr(:);
    x_ref  = x_ref(:);

    N = nmpc.N;
    nx = nmpc.n_states;
    nu = nmpc.n_controls;
    Ueq = nmpc.U_ref;

    % parameters: [x0; x_ref]
    P = [x_curr; x_ref];

    % set initial guess
    X_init_mat = repmat(x_curr, 1, N+1); % (nx x (N+1))
    X_init = reshape(X_init_mat, (N+1)*nx, 1); % (nx*(N+1) x 1)

    % control guess
    U_init_mat = repmat(Ueq, 1, N); % (nu x N)
    U_init = reshape(U_init_mat, nu*N, 1); % (nu*N

    Z_init = [X_init; U_init];

    % Call NMPC solver
    sol = nmpc.solver(...
        'x0', Z_init, ...
        'lbx', nmpc.args.lbx, ...
        'ubx', nmpc.args.ubx, ...
        'lbg', nmpc.args.lbg, ...
        'ubg', nmpc.args.ubg, ...
        'p', P);

    Z_opt = full(sol.x);

    % extract optimal solution for states and control
    nxN1 = nx*(N+1);
    X_opt_vec = Z_opt(1:nxN1);
    U_opt_vec = Z_opt(nxN1+1:end);

    X_opt = reshape(X_opt_vec, nx, N+1); % (nx x (N+1))
    U_opt = reshape(U_opt_vec, nu, N);   % (nu x N)

    % apply the first control
    u_applied = U_opt(:, 1);

    % some other info for debugging if need
    aux = struct;
    aux.X_opt = X_opt;
    aux.U_opt = U_opt;
    aux.solver_stats = nmpc.solver.stats();

end