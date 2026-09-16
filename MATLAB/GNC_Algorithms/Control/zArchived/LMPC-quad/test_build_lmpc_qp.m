% filename: MATLAB/GNC_Algorithms/Control/LMPC/test_build_lmpc_qp.m
%
% Unit test for build_lmpc_qp.m
% - Uses a small toy system (double integrator)
% - Checks sizes of H, Aeq, beq, lb, ub
% - Verifies that dynamics constraints Aeq*z = beq are satisfied
%   for a manually constructed feasible trajectory
% - Optionally solves one QP with quadprog

clear; clc;

fprintf('=== LMPC QP Builder Unit Test ===\n');

%% 1. Toy system definition (2-state, 1-input, N=3)

Ts = 0.1;
A_d = [1 Ts;
       0  1];
B_d = [0;
       Ts];

[nx, nu] = size(B_d);
N   = 3;  % prediction horizon

% Cost weights
Q = diag([10, 1]);
R = 0.1;
P = Q;  % simple choice

% Initial state
x0 = [1; 0];

% State and input bounds
x_min = [-5; -5];
x_max = [ 5;  5];
u_min = -1;
u_max =  1;

%% 2. Build QP matrices
[H, f, Aeq, beq, lb, ub, idx] = build_lmpc_qp( ...
    A_d, B_d, Q, R, P, N, x0, x_min, x_max, u_min, u_max);

nz_x = nx*(N+1);
nz_u = nu*N;
nz   = nz_x + nz_u;

fprintf('nz      = %d\n', nz);
fprintf('size(H) = [%d %d]\n', size(H,1), size(H,2));
fprintf('size(Aeq) = [%d %d]\n', size(Aeq,1), size(Aeq,2));
fprintf('size(beq) = [%d %d]\n', size(beq,1), size(beq,2));
fprintf('size(lb)  = [%d %d]\n', size(lb,1), size(lb,2));
fprintf('size(ub)  = [%d %d]\n', size(ub,1), size(ub,2));

assert(isequal(size(H),   [nz, nz]),   'H has wrong size');
assert(isequal(size(Aeq), [ (N+1)*nx, nz ]), 'Aeq has wrong size');
assert(isequal(size(beq), [ (N+1)*nx, 1 ]),  'beq has wrong size');
assert(isequal(size(lb),  [nz, 1]),   'lb has wrong size');
assert(isequal(size(ub),  [nz, 1]),   'ub has wrong size');

fprintf('✔ Dimension checks passed.\n');

%% 3. Build a feasible trajectory by hand and check Aeq*z = beq

% Choose some arbitrary input sequence (within bounds)
u_seq = [0.2; 0.0; -0.1];   % 3 steps (N=3)

% Forward propagate to get x_k
x = zeros(nx, N+1);
x(:,1) = x0;
for k = 1:N
    x(:,k+1) = A_d * x(:,k) + B_d * u_seq(k);
end

% Stack into z = [x0;x1;...;xN; u0;...;u_{N-1}]
z = zeros(nz,1);
for k = 0:N
    z(idx.states(k)) = x(:,k+1);   % x(:,1)=x0, so use k+1
end
for k = 0:N-1
    z(idx.inputs(k)) = u_seq(k+1);
end

% Check dynamics constraints
dyn_resid = Aeq*z - beq;
fprintf('norm(Aeq*z - beq) = %.3e\n', norm(dyn_resid));

assert(norm(dyn_resid) < 1e-10, 'Dynamics constraints are not satisfied!');
fprintf('✔ Dynamics constraints test passed.\n');

%% 4. Check bounds: lb <= z <= ub
viol_lb = find(z < lb - 1e-9);
viol_ub = find(z > ub + 1e-9);

if isempty(viol_lb) && isempty(viol_ub)
    fprintf('✔ Feasible trajectory respects bounds.\n');
else
    warning('Some components of z violate bounds.');
end

%% 5. Optional: solve the QP with quadprog

use_quadprog = true;

if use_quadprog
    fprintf('Solving toy QP with quadprog...\n');
    opts = optimoptions('quadprog', 'Display','off', ...
                        'Algorithm','interior-point-convex');
    [z_opt, fval, exitflag, output] = quadprog(H, f, [], [], Aeq, beq, lb, ub, [], opts);

    fprintf('quadprog exitflag = %d\n', exitflag);
    if exitflag == 1
        fprintf('✔ QP solved successfully. fval = %.4f\n', fval);
        u0_opt = z_opt(idx.inputs(0));
        fprintf('u0_opt = %.4f\n', u0_opt);
    else
        warning('quadprog did not converge normally.');
    end
end

fprintf('=== LMPC QP Builder Unit Test COMPLETE ===\n');


% quick quad-like dimension test (no dynamics semantics, just sizes)
nx = 13; nu = 4; N = 10;
A_d = eye(nx);            % dummy but stable
B_d = [zeros(9,4); eye(4)];  % dummy input mapping

Q_pos  = 5.0 * eye(3);
Q_vel  = 1.0 * eye(3);
Q_quat = 2.0 * eye(4);
Q_omg  = 0.5 * eye(3);
Q = blkdiag(Q_pos, Q_vel, Q_quat, Q_omg);
R = 1e-4 * eye(nu);
P = Q;

x0 = zeros(nx,1); x0(7) = 1; % hover-like

x_max = [ 3; 3; 0.5; 2; 2; 2; 1; 1; 1; 1; 2; 2; 2];
x_min = -x_max;
dT_max = 10; tau_max = 3;
u_min = [-dT_max; -tau_max; -tau_max; -tau_max];
u_max = [ dT_max;  tau_max;  tau_max;  tau_max];

[H,f,Aeq,beq,lb,ub,idx] = build_lmpc_qp(A_d,B_d,Q,R,P,N,x0,x_min,x_max,u_min,u_max);
