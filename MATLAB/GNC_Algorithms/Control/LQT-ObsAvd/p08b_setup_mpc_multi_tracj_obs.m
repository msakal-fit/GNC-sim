% filename: MATLAB/GNC_Algorithms/Control/NMPC/p08b_setup_mpc_multi_tracj_obs.m
%
% NMPC with multiple shooting

function nmpc = p08b_setup_mpc_multi_tracj_obs(px4_config, Ts, N)
    % add path
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

    % Symbolic state and input variables
    % State: [r(3); v(3); q(4); omega(3)]
    x = SX.sym('x', n_states, 1);
    u = SX.sym('u', n_controls, 1);

    r     = x(1:3);
    v     = x(4:6);
    q     = x(7:10);
    omega = x(11:13);

    % Controls
    U1 = u(1);  % total thrust
    U2 = u(2);  % tau_x
    U3 = u(3);  % tau_y
    U4 = u(4);  % tau_z

    % Quaternions
    q0 = q(1);
    q1 = q(2);
    q2 = q(3);
    q3 = q(4);

    % Angular rates
    p_rate = omega(1);
    q_rate = omega(2);
    r_rate = omega(3);

    % Nonlinear dynamics (quadcopter)
    % Position derivative
    rdot = v;

    % Translational dynamics
    vxdot = -(U1/m) * 2*(q1*q3 + q0*q2);
    vydot = -(U1/m) * 2*(q2*q3 - q0*q1);
    vzdot =  g      - (U1/m) * (q0^2 - q1^2 - q2^2 + q3^2);
    vdot  = [vxdot; vydot; vzdot];

    % Quaternion kinematics
    q0dot = 0.5 * (-p_rate*q1 - q_rate*q2 - r_rate*q3);
    q1dot = 0.5 * ( p_rate*q0 + r_rate*q2 - q_rate*q3);
    q2dot = 0.5 * ( q_rate*q0 - r_rate*q1 + p_rate*q3);
    q3dot = 0.5 * ( r_rate*q0 + q_rate*q1 - p_rate*q2);
    qdot  = [q0dot; q1dot; q2dot; q3dot];

    % Rotational dynamics
    pdot      = (1/Jx) * (U2 + (Jy - Jz)*q_rate*r_rate);
    qrdot     = (1/Jy) * (U3 + (Jz - Jx)*p_rate*r_rate);
    rdot_ang  = (1/Jz) * (U4 + (Jx - Jy)*p_rate*q_rate);
    omegadot  = [pdot; qrdot; rdot_ang];

    % Full state derivative
    xdot = [rdot; vdot; qdot; omegadot];

    % CasADi function f(x,u)
    f = Function('f', {x, u}, {xdot});

    % one step integration using RK4
    Xk_sym = SX.sym('Xk', n_states, 1);
    Uk_sym = SX.sym('Uk', n_controls, 1);

    k1 = f(Xk_sym, Uk_sym);
    k2 = f(Xk_sym + (Ts/2)*k1,  Uk_sym);
    k3 = f(Xk_sym + (Ts/2)*k2,  Uk_sym);
    k4 = f(Xk_sym + Ts*k3,      Uk_sym);

    Xk_next = Xk_sym + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4);
    F_RK4   = Function('F_RK4', {Xk_sym, Uk_sym}, {Xk_next});


    % setup cost function
    Q_pos = diag([ 200, 200, 10 ]);
    Q_vel = diag([ 2,  2,  2  ]);
    Q_q   = diag([ 10, 10, 10, 10 ]);
    Q_om  = diag([ 1,  1,  1 ]);

    Q  = blkdiag(Q_pos, Q_vel, Q_q, Q_om);
    Qf = Q;    % terminal weight

    % Input weight
    R = diag([ 0.3, 0.1, 0.1, 0.1 ]);

    % Hover input
    U_ref = [T_hover; 0; 0; 0];

    % add obstacle setup
    obs_center_3d = [8.0; 0.0; -5.0]; 

    obs_radius = 0.25;        % radius

    % min separation distance
    min_separation = 1.5; % [m]

    % Safe radius
    % d >= obs_radius + min_separation
    R_safe = obs_radius + min_separation;


    % multiple shooting decision
    % control
    U_var = SX.sym('U', n_controls, N);

    % states
    X_var = SX.sym('X', n_states, N+1);

    % Parameters: 
    % [x0; x_ref^(1); ...; x_ref^(N); u_prev]
    P = SX.sym('P', n_states*(N+1) + n_controls, 1);

    x0_sym   = P(1:n_states);
    u_prev_param = P(n_states*(N+1)+1 : end);

    % reference trajectory as matrix (n_states x (N))
    Xref = reshape(P(n_states+1:n_states*(N+1)), n_states, N);

    % build objective
    obj = 0;
    %g = [];

    g_dynamics = [];
    g_obstacle = [];

    % initial condition constraint 
    % X(:, 1) = x0
    g_dynamics = [g_dynamics; X_var(:, 1) - x0_sym];

    % stage cost + dynamics constraints
    for k = 1:N
        Xk = X_var(:, k);
        Uk = U_var(:, k);
        Xk1 = X_var(:, k+1);

        xref_k = Xref(:, k);

        % stage cost
        x_err = Xk - xref_k;
        u_err = Uk - U_ref;

        % add input smoothness
        if k == 1
            du = Uk - u_prev_param;
        else
            du = Uk - U_var(:, k-1);
        end

        obj = obj + x_err.'*Q*x_err + u_err.'*R*u_err + du.'*R*du;

        % dynamics constraint
        % X_k1 - F_RK4(X_k, U_k) = 0
        Xk_next = F_RK4(Xk, Uk);
        g_dynamics = [g_dynamics; Xk1 - Xk_next];


        % obstacle avoidance constraint
        px_k = Xk(1);
        py_k = Xk(2);
        pz_k = Xk(3);

        % distance^2 from obstacle center
        d2_k = (px_k - obs_center_3d(1))^2 + ...
               (py_k - obs_center_3d(2))^2 + ...
               (pz_k - obs_center_3d(3))^2;

        % setup px, px to be outside of radius R_safe
        g_obs_k = R_safe^2 - d2_k;
        g_obstacle = [g_obstacle; g_obs_k];
    end

    % add terminal cost on the final state
    xref_N = Xref(:, N);
    x_err_N = X_var(:, N+1) - xref_N;
    obj     = obj + x_err_N.'*Qf*x_err_N;

    % obstacle constraint at terminal state X(:, N+1)
    XN  = X_var(:, N+1);
    pxN = XN(1);
    pyN = XN(2);
    pzN = XN(3);

    d2_N    = (pxN - obs_center_3d(1))^2 + ...
              (pyN - obs_center_3d(2))^2 + ...
              (pzN - obs_center_3d(3))^2;
    g_obs_N = R_safe^2 - d2_N;
    g_obstacle   = [g_obstacle; g_obs_N];

    % combine all constraints
    g = [g_dynamics; g_obstacle];

    % make the decision variables into a single column vector
    OPT_X = reshape(X_var, n_states*(N+1), 1);
    OPT_U = reshape(U_var, n_controls*N, 1);

    OPT_variables = [OPT_X; OPT_U];

    % NLP structure
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

    % add bounds
    % number of decision variables
    n_dec = n_states*(N+1) + n_controls*N;

    lbx = -inf(n_dec, 1);
    ubx = inf(n_dec, 1);

    % no bounds on states

    % add bound for inputs
    base = n_states*(N+1);
    for k = 1:N
        idx = base + (k-1)*n_controls + (1:n_controls);
        lbx(idx) = [T_min; -tau_max; -tau_max; -tau_max];
        ubx(idx) = [T_max;  tau_max;  tau_max;  tau_max];
    end

    % number of equality (dynamics) constraints
    n_dyn_constr = (N+1)*n_states;   % X0 + N dynamics steps

    % number of obstacle constraints: one per stage k=1..N and one at terminal
    n_obs_constr = (N+1);            % adjust if you also add at k=0

    % equality constraints: g_dyn == 0
    lbg_dyn = zeros(n_dyn_constr, 1);
    ubg_dyn = zeros(n_dyn_constr, 1);

    % obstacle constraints: g_obs <= 0
    %   g_obs = R_safe^2 - d^2 <= 0  ⇒  d^2 >= R_safe^2
    lbg_obs = -inf(n_obs_constr, 1);
    ubg_obs = zeros(n_obs_constr, 1);

    % full bounds must match [g_dyn; g_obs]
    lbg = [lbg_dyn; lbg_obs];
    ubg = [ubg_dyn; ubg_obs];

    nmpc.solver = solver;
    nmpc.F_RK4     = F_RK4;
    nmpc.Ts = Ts;
    nmpc.N = N;
    nmpc.n_states = n_states;
    nmpc.n_controls = n_controls;

    nmpc.args = struct;
    nmpc.args.lbx = lbx;
    nmpc.args.ubx = ubx;
    nmpc.args.lbg = lbg;
    nmpc.args.ubg = ubg;

    % obstacle information
    nmpc.obs_center3d = obs_center_3d;
    nmpc.obs_radius = obs_radius;
    nmpc.min_sep    = min_separation;
    nmpc.R_safe     = R_safe;

    % hover input for warm start
    % make it easy to find the solution
    nmpc.U_ref = U_ref;

end