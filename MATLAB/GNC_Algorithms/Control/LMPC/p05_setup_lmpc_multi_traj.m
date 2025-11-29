% filename: MATLAB/GNC_Algorithms/Control/LMPC/p05_setup_lmpc_multi_traj.m

function lmpc = p05_setup_lmpc_multi_traj(px4_config, Ts, N, Ad, Bd)

    % Add CasADi and import
    addpath('/home/msakal2024@fit.edu/TKRM/GNC-sim/casadi-3.7.2-linux64-matlab2018b');
    import casadi.*

    n_states    = 13;
    n_controls  = 4;

    % physical parameters from quadcopter model
    m  = px4_config.m;
    g  = px4_config.g;


         % Hover thrust 
    T_hover = m * g;

    % input bounds
    T_min = 0.5 * T_hover;
    T_max = 1.5 * T_hover;

    % Torques bounds
    tau_max = 0.2;   % [N·m]


    % Setup cost function weights
    Q_pos = diag([ 200, 200,  50 ]);
    Q_vel = diag([   2,   2,   2 ]);
    Q_q   = diag([  20,  20,  20, 20 ]);
    Q_om  = diag([   2,   2,   2 ]);

    Q  = blkdiag(Q_pos, Q_vel, Q_q, Q_om);
    Qf = Q; % terminal weight

    % Input weight
    R = diag([ 2,  1,  1,  1 ]);


    % input hover
    U_ref = [T_hover; 0; 0; 0];

    % decision variable (multiple shooting)
    % controls U(:, 1...N)
    U_var = SX.sym('U_var', n_controls, N); % (nu x N)
    % states X(:, 1...N+1)
    X_var = SX.sym('X_var', n_states, N+1);  % (nx x (N+1))


    % Parameters: [x0; x_ref^(1); ...; x_ref^(N); u_prev]
    P = SX.sym('P', n_states*(N+1) + n_controls, 1);

    x0_sym       = P(1:n_states);
    u_prev_param = P(n_states*(N+1)+1 : end);

    % reference trajectory (n_states x N)
    Xref = reshape(P(n_states+1 : n_states*(N+1)), n_states, N);


    % Build objective and constraints
    obj = 0;
    g   = [];

    % Initial condition constraint
    g = [g; X_var(:, 1) - x0_sym];


    % Stage cost + dynamics
    for k = 1:N
        Xk  = X_var(:, k);
        Uk  = U_var(:, k);
        Xk1 = X_var(:, k+1);

        xref_k = Xref(:, k);

        % tracking error
        x_err = Xk - xref_k;
        u_err = Uk - U_ref;

        % input smoothness to penalize large change of u
        if k == 1
            du = Uk - u_prev_param;
        else
            du = Uk - U_var(:, k-1);
        end

        obj = obj + x_err.'*Q*x_err + u_err.'*R*u_err + du.'*R*du;

        % Linear discrete-time dynamics
        Xk_next = Ad * Xk + Bd * Uk;

        % Dynamics constraint: X_{k+1} - (Ad X_k + Bd U_k) = 0
        g = [g; Xk1 - Xk_next];
    end

    % Terminal cost
    xref_N  = Xref(:, N);
    x_err_N = X_var(:, N+1) - xref_N;
    obj     = obj + x_err_N.'*Qf*x_err_N;

    % convert to column vector

    OPT_X = reshape(X_var, n_states*(N+1), 1);
    OPT_U = reshape(U_var, n_controls*N, 1);
    OPT_variables = [OPT_X; OPT_U];

    % NLP definition
    nlp_prob = struct('f', obj, ...
                      'x', OPT_variables, ...
                      'g', g, ...
                      'p', P);

    % IPOPT solver options
    opts = struct();
    opts.ipopt.max_iter = 200;
    opts.ipopt.print_level = 0;
    opts.print_time = 0;
    opts.ipopt.acceptable_tol = 1e-8;
    opts.ipopt.acceptable_obj_change_tol = 1e-6;

    solver = nlpsol('solver', 'ipopt', nlp_prob, opts);

    % === Bounds ===
    n_dec = n_states*(N+1) + n_controls*N;
    lbx = -inf(n_dec, 1);
    ubx =  inf(n_dec, 1);


    % Input bounds only (states unbounded here)
    base_u = n_states*(N+1);
    for k = 1:N
        idx = base_u + (k-1)*n_controls + (1:n_controls);
        lbx(idx) = [T_min; -tau_max; -tau_max; -tau_max];
        ubx(idx) = [T_max;  tau_max;  tau_max;  tau_max];
    end

    % Equality constraints bounds (initial + dynamics)
    n_constr = n_states * (N+1);
    lbg = zeros(n_constr, 1);
    ubg = zeros(n_constr, 1);

    % Pack into LMPC struct (same fields as nmpc for reuse)
    lmpc.solver      = solver;
    lmpc.Ts          = Ts;
    lmpc.N           = N;
    lmpc.n_states    = n_states;
    lmpc.n_controls  = n_controls;
    lmpc.U_ref       = U_ref;

    lmpc.args        = struct;
    lmpc.args.lbx    = lbx;
    lmpc.args.ubx    = ubx;
    lmpc.args.lbg    = lbg;
    lmpc.args.ubg    = ubg;
end


