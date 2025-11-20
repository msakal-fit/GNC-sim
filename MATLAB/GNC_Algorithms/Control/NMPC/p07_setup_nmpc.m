% filename: MATLAB/GNC_Algorithms/Control/NMPC/p07_setup_nmpc.m

% Minimal NMPC setup for quadcopter hover (Problem 7)

function nmpc = p07_setup_nmpc(px4_config, Ts, N)

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


    % Setup cost function weights
    % [ px py pz  vx vy vz  q0 q1 q2 q3  p q r ]
    Q_pos = diag([  50,   50,  50]);
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


    % single shooting NMPC setup
    % Controls over horizon
    U = SX.sym('U', n_controls, N);

    % Parameters: 
    % [x0; x_ref^(1);
    %    ...
    %   x_ref^(N);
    %   u_prev]
    P = SX.sym('P', n_states*(N+1) + n_controls, 1);

    x0_sym   = P(1:n_states);
    u_prev_param = P(n_states*(N+1)+1 : end);


    Xk  = x0_sym;   % current predicted state
    obj = 0;        % objective
    g   = [];

    for k = 1:N
        uk = U(:,k);

        % extract reference for stage k
        idx_start = n_states*k + 1;
        idx_end   = n_states*(k+1);
        xref_k    = P(idx_start:idx_end);

        % Tracking errors
        x_err = Xk - xref_k;
        u_err = uk - U_ref;

        % Input smoothness Δu
        if k == 1
            du = uk - u_prev_param;
        else
            du = uk - U(:,k-1);
        end

        % Stage cost
        obj = obj ...
            + x_err.'*Q*x_err ...
            + u_err.'*R*u_err ...
            + du.'*S*du;

        % RK4 integration
        k1 = f(Xk,               uk);
        k2 = f(Xk + (Ts/2)*k1,   uk);
        k3 = f(Xk + (Ts/2)*k2,   uk);
        k4 = f(Xk + Ts*k3,       uk);

        Xk = Xk + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4);
    end

    % Terminal cost on final state using x_ref^(N)
    xref_N = P(n_states*N + 1 : n_states*(N+1));
    x_err_N = Xk - xref_N;
    obj     = obj + x_err_N.'*Qf*x_err_N;

    % Flatten decision variables into a single column vector
    OPT_variables = reshape(U, n_controls*N, 1);

    % NLP structure: unconstrained, with parameters P
    nlp_prob = struct('f', obj, ...
                    'x', OPT_variables, ...
                    'g', [], ...
                    'p', P);

    % Create NLP solver, IPOPT
    opts = struct;
    opts.ipopt.max_iter                  = 200;
    opts.ipopt.print_level               = 0;
    opts.print_time                      = 0;
    opts.ipopt.acceptable_tol            = 1e-8;
    opts.ipopt.acceptable_obj_change_tol = 1e-6;

    solver = nlpsol('solver', 'ipopt', nlp_prob, opts);

    % apply input bounds

    lbx = zeros(n_controls*N, 1);
    ubx = zeros(n_controls*N, 1);

    for k = 1:N
        idx = (k-1)*n_controls + (1:n_controls);
        lbx(idx) = [ T_min; -tau_max; -tau_max; -tau_max ];
        ubx(idx) = [ T_max;  tau_max;  tau_max;  tau_max ];
    end

    % Pack everything into a struct
    nmpc.solver      = solver;
    nmpc.Ts          = Ts;
    nmpc.N           = N;
    nmpc.n_states    = n_states;
    nmpc.n_controls  = n_controls;

    nmpc.args        = struct;
    nmpc.args.lbx    = lbx;
    nmpc.args.ubx    = ubx;
    nmpc.args.lbg    = [];
    nmpc.args.ubg    = [];


    % hover input for warm start
    nmpc.U_ref       = U_ref;

end