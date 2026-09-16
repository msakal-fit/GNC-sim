% filename: MATLAB/GNC_Algorithms/Control/LMPC/build_lmpc_qp.m
%
% Build the LMPC QP matrices (non-condensed form):
%
%   z = [x0; x1; ...; xN; u0; ...; u_{N-1}]
%
% Cost:
%   J = sum_{k=0}^{N-1} x_k' Q x_k + u_k' R u_k + x_N' P x_N
%
% Constraints:
%   x0 = x_current
%   x_{k+1} = A_d x_k + B_d u_k
%   x_min <= x_k <= x_max
%   u_min <= u_k <= u_max

function [H, f, Aeq, beq, lb, ub, idx] = build_lmpc_qp( ...
    A_d, B_d, Q, R, P, N, x0, x_min, x_max, u_min, u_max)

    % Dimensions
    [nx, nu] = size(B_d);

    % (debug)
    %fprintf('LMPC: nx=%d, nu=%d, size(R)=[%d %d]\n', nx, nu, size(R,1), size(R,2));

    % ---------------------------------------------------------------------
    % Ensure Q, R, P have correct dimensions
    % ---------------------------------------------------------------------
    % Q: either scalar, vector (length nx), or nx x nx
    if isscalar(Q)
        Q = Q * eye(nx);
    elseif isvector(Q)
        Q = diag(Q(:));
    end
    % R: either scalar, vector (length nu), or nu x nu
    if isscalar(R)
        R = R * eye(nu);
    elseif isvector(R)
        R = diag(R(:));
    end
    % P: either scalar, vector (length nx), or nx x nx
    if isscalar(P)
        P = P * eye(nx);
    elseif isvector(P)
        P = diag(P(:));
    end

    % Sanity checks
    assert(all(size(Q) == [nx, nx]), 'Q must be nx x nx');
    assert(all(size(R) == [nu, nu]), 'R must be nu x nu');
    assert(all(size(P) == [nx, nx]), 'P must be nx x nx');

    % ---------------------------------------------------------------------
    % Decision vector sizes
    % ---------------------------------------------------------------------
    nz_x = nx * (N + 1); % number of state variables
    nz_u = nu * N;       % number of control variables
    nz   = nz_x + nz_u;  % total number of decision variables

    offset_u = nz_x;     % index offset for control variables in z

    % ---------------------------------------------------------------------
    % 1. Quadratic cost: 0.5 z' H z + f' z
    % ---------------------------------------------------------------------

    % Build state cost block Hx = blkdiag(Q, ..., Q, P)
    Q_blocks = cell(1, N+1);
    for k = 1:N
        Q_blocks{k} = Q;
    end
    Q_blocks{N+1} = P;
    Hx = blkdiag(Q_blocks{:});   % size (nz_x x nz_x)

    % Build input cost block Hu = blkdiag(R, ..., R)
    if N > 0
        R_blocks = cell(1, N);
        for k = 1:N
            R_blocks{k} = R;
        end
        Hu = blkdiag(R_blocks{:});   % size (nz_u x nz_u)
    else
        Hu = [];   % no inputs => empty
    end

    % Assemble full H as block diagonal of [Hx, Hu]
    H = blkdiag(Hx, Hu);          % size (nz x nz)
    f = zeros(nz, 1);             % no linear term

    % ---------------------------------------------------------------------
    % 2. Equality constraints: Aeq * z = beq
    %    - initial condition: x0 = x_current
    %    - dynamics: x_{k+1} - A_d x_k - B_d u_k = 0
    % ---------------------------------------------------------------------
    neq = (N+1)*nx;      % nx for x0 + nx for each dynamics step
    Aeq = zeros(neq, nz);
    beq = zeros(neq, 1);

    row = 1;

    % (a) Initial condition: x0 = x_current
    idx_x0 = 1:nx;
    Aeq(row:row+nx-1, idx_x0) = eye(nx);
    beq(row:row+nx-1) = x0(:);
    row = row + nx;

    % (b) Dynamics for k = 0..N-1
    for k = 0:N-1
        idx_xk   = (k*nx+1):(k*nx+nx);
        idx_xkp1 = ((k+1)*nx+1):((k+1)*nx+nx);

        % u_k indices: [offset_u + k*nu + 1 ... offset_u + (k+1)*nu]
        idx_uk   = (offset_u + k*nu + 1):(offset_u + (k+1)*nu);

        % x_{k+1} - A_d x_k - B_d u_k = 0
        Aeq(row:row+nx-1, idx_xkp1) =  eye(nx);
        Aeq(row:row+nx-1, idx_xk)   = -A_d;
        Aeq(row:row+nx-1, idx_uk)   = -B_d;
        row = row + nx;
    end

    % ---------------------------------------------------------------------
    % 3. Variable bounds: lb <= z <= ub
    %    (we use simple box constraints instead of Aineq/bineq)
    % ---------------------------------------------------------------------
    lb = -inf(nz, 1);
    ub =  inf(nz, 1);

    % State bounds x_min <= x_k <= x_max
    if ~isempty(x_min) && ~isempty(x_max)
        x_min = x_min(:);
        x_max = x_max(:);
        for k = 0:N
            idx_x = (k*nx+1):(k*nx+nx);
            lb(idx_x) = x_min;
            ub(idx_x) = x_max;
        end
    end

    % Input bounds u_min <= u_k <= u_max
    if ~isempty(u_min) && ~isempty(u_max)
        u_min = u_min(:);
        u_max = u_max(:);
        for k = 0:N-1
            idx_u = (offset_u + k*nu + 1):(offset_u + (k+1)*nu);
            lb(idx_u) = u_min;
            ub(idx_u) = u_max;
        end
    end

    % Index access for states and inputs (handy for unpacking)
    idx.states = @(k) (k*nx+1):(k*nx+nx);                       % x_k
    idx.inputs = @(k) (offset_u + k*nu + 1):(offset_u + (k+1)*nu);  % u_k

end
