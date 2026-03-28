% filename: MATLAB/GNC_Algorithms/Control/NMPC/p08b_mpc_step_multi_traj.m

% NMPC step with multiple shooting

function [u_applied, aux] = p08b_mpc_step_multi_traj(x_curr, Xref_hor, u_prev, nmpc)
    x_curr = x_curr(:);
    u_prev = u_prev(:);
    %x_ref  = x_ref(:);

    N = nmpc.N;
    nx = nmpc.n_states;
    nu = nmpc.n_controls;
    Uref = nmpc.U_ref;

    % parameters: [x0; x_ref^(1); ...; x_ref^(N); u_prev]
    P = zeros(nx*(N+1) + nu, 1);

    % current state
    P(1:nx) = x_curr;

    % horizon reference
    for k = 1:N
        idx = nx*k + (1:nx);
        P(idx) = Xref_hor(:,k);
    end

    % previous control
    P(nx*(N+1)+1 : end) = u_prev;

    % set initial guess
    X_init_mat = repmat(x_curr, 1, N+1); % (nx x (N+1))
    X_init = reshape(X_init_mat, nx*(N+1), 1); % (nx*(N+1) x 1)

    % control guess
    U_init_mat = repmat(Uref, 1, N); % (nu x N)
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