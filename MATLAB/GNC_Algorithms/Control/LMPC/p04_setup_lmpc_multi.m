% filename: MATLAB/GNC_Algorithms/Control/LMPC/p04_setup_lmpc_multi.m


function lmpc = p04_setup_lmpc_multi(px4_config, Ts, N, Ad, Bd)

    % Add CasADi and import
    addpath('/home/msakal2024@fit.edu/TKRM/GNC-sim/casadi-3.7.2-linux64-matlab2018b');
    import casadi.*

    n_states    = 13;
    n_controls  = 4;

    % physical parameters from quadcopter model
    m  = px4_config.m;
    g  = px4_config.g;
    J  = px4_config.inertia;
    Jx = J(1,1);
    Jy = J(2,2);
    Jz = J(3,3);

     % Hover thrust 
    T_hover = m * g;

    % input bounds
    T_min = 0.5 * T_hover;
    T_max = 1.5 * T_hover;

    % Torques bounds
    tau_max = 0.2;   % [N·m]

    % state bounds
    pos_min = [-10, -10, -10];
    pos_max = [ 10,  10,   0];

    vel_min = [-5, -5, -5];
    vel_max = [ 5,  5,  5];

    % quat & rates bounds are left unbounded

    % Setup cost function weights
    % [ px py pz  vx vy vz  q0 q1 q2 q3  p q r ]
    Q_pos = diag([  200,   200,  50]);
    Q_vel = diag([  2,   2,   2]);
    Q_q   = diag([ 20,  20,  20, 20]);
    Q_om  = diag([  2,   2,   2]);

    Q  = blkdiag(Q_pos, Q_vel, Q_q, Q_om);
    Qf = Q; % terminal weigth = state weight

    % Input weight
    R = diag([ 0.3,  0.1,  0.1,  0.1 ]);

    % Smoothness weight
    S = diag([ 0.1, 0.05, 0.05, 0.05 ]);

    % Hover input
    U_ref = [T_hover; 0; 0; 0];


    % decision variable (multiple shooting)
    % controls U(:, 1...N)
    U_var = SX.sym('U_var', n_controls, N); % (nu x N)
    % states X(:, 1...N+1)
    X_var = SX.sym('X_var', n_states, N+1);  % (nx x (N+1))

    % Parameters: [x0; x_ref]
    P = SX.sym('P', n_states + n_states, 1);
    x0_sym = P(1:n_states);
    x_ref_sym = P(n_states+1:end);

    % build objective
    obj = 0;
    g = [];

    % initial condition constraint
    g = [g; X_var(:, 1) - x0_sym];

    % LMPC state cosst
    for k = 1:N
        Xk = X_var(:, k);
        Uk = U_var(:, k);
        Xk1 = X_var(:, k+1);

        % stage cost
        x_err = Xk - x_ref_sym;
        u_err = Uk - U_ref;

        obj = obj + x_err.'*Q*x_err + u_err.'*R*u_err;

        % dynamics constraint
        % X(:, k+1) - Ad*X(:, k) - Bd*U(:, k) = 0
        
        % around the reference
        % X_{k+1} = x_ref + Ad (X_k - x_ref) + Bd (U_k - U_ref)
        Xk_next = x_ref_sym + Ad * (Xk - x_ref_sym) + Bd * (Uk - U_ref);

        %Xk_next = Ad * Xk + Bd * Uk;
        g = [g; Xk1 - Xk_next];
    end

    % terminal cost
    x_err_N = X_var(:, N+1) - x_ref_sym;
    obj     = obj + x_err_N.'*Qf*x_err_N;


    % convert decision variables to single column vector
    OPT_X = reshape(X_var, n_states*(N+1), 1); % state
    OPT_U = reshape(U_var, n_controls*N, 1);   % control
    OPT_variables = [OPT_X; OPT_U];

    % --- NLP setup ---
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

    % state bounds (pos, vel)
    n_dec = n_states*(N+1) + n_controls*N;
    lbx = -inf * ones(n_dec, 1);
    ubx =  inf * ones(n_dec, 1);


    % X_var: [X(:,1), ..., X(:,N+1)]
    for k = 1:(N+1)
        base_x = (k-1)*n_states;

        % position
        lbx(base_x + (1:3)) = pos_min;
        ubx(base_x + (1:3)) = pos_max;

        % velocity
        lbx(base_x + (4:6)) = vel_min;
        ubx(base_x + (4:6)) = vel_max;
    end

    % input bounds
    base_u = n_states*(N+1);
    for k = 1:N
        idx = base_u + (k-1)*n_controls + (1:n_controls);
        lbx(idx) = [T_min; -tau_max; -tau_max; -tau_max];
        ubx(idx) = [T_max;  tau_max;  tau_max;  tau_max];
    end 

    % equality constraints bounds
    n_constr = n_states * (N+1); % inital cond + dynamics
    lbg = zeros(n_constr, 1);
    ubg = zeros(n_constr, 1);

    % pack into lmpc struct
    lmpc.solver = solver;
    lmpc.Ts = Ts;
    lmpc.N = N;
    lmpc.n_states = n_states;
    lmpc.n_controls = n_controls;

    lmpc.args = struct;
    lmpc.args.lbx = lbx;
    lmpc.args.ubx = ubx;
    lmpc.args.lbg = lbg;
    lmpc.args.ubg = ubg;

    lmpc.U_ref = U_ref;
end





