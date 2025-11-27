% filename: MATLAB/GNC_Algorithms/Control/tests/test_drone_linearization_positions.m
%
% Test that linearization at different positions (same hover attitude)
% gives the same A, B matrices for the quadcopter model.

clear; clc;

% Load quadcopter parameters
px4_config = get_x500_params();
m = px4_config.m;
g = px4_config.g;

% --- Equilibrium input (hover) ---
u_eq = [m * g; 0; 0; 0];

% --- Equilibrium states at two different positions ---
% State: [r(3); v(3); q(4); omega(3)]

x_eq_A = zeros(13,1);
x_eq_A(1:3)   = [0; 0; -5];    % position A
x_eq_A(4:6)   = [0; 0; 0];     % velocity
x_eq_A(7:10)  = [1; 0; 0; 0];  % level attitude
x_eq_A(11:13) = [0; 0; 0];     % body rates

x_eq_B = zeros(13,1);
x_eq_B(1:3)   = [2; 2; -3];    % position B
x_eq_B(4:6)   = [0; 0; 0];     % velocity
x_eq_B(7:10)  = [1; 0; 0; 0];  % same attitude
x_eq_B(11:13) = [0; 0; 0];     % body rates

% --- Compute linearizations ---
[A_A, B_A] = drone_linear_dynamics(x_eq_A, u_eq, px4_config);
[A_B, B_B] = drone_linear_dynamics(x_eq_B, u_eq, px4_config);

% --- Compare ---
diffA = A_A - A_B;
diffB = B_A - B_B;

maxA = max(abs(diffA(:)));
maxB = max(abs(diffB(:)));

fprintf('Max abs difference in A: %e\n', maxA);
fprintf('Max abs difference in B: %e\n', maxB);

% Optional: assert they are (numerically) identical within a tolerance
tol = 1e-9;
if maxA < tol && maxB < tol
    fprintf('PASS: Linearizations at A and B are the same (within tol = %g).\n', tol);
else
    fprintf('WARNING: Differences exceed tolerance %g.\n', tol);
end

% Extra check: random positions with same hover attitude
for i = 1:5
    r_rand = [10; 10; -10] .* (2*rand(3,1) - 1); % random XYZ
    x_eq_rand = x_eq_A;
    x_eq_rand(1:3) = r_rand;

    [A_rand, B_rand] = drone_linear_dynamics(x_eq_rand, u_eq, px4_config);

    maxA_rand = max(abs(A_rand(:) - A_A(:)));
    maxB_rand = max(abs(B_rand(:) - B_A(:)));

    fprintf('Test %d: max|A_rand - A_A| = %e, max|B_rand - B_A| = %e\n', ...
            i, maxA_rand, maxB_rand);
end

