function [H, rhs, diagInfo] = buildCostMatrices(F, Phi, x0, xTargetVec, Q, R, S, Np, Nc)
%BUILDCOSTMATRICES Build the unconstrained-MPC normal-equation matrices
%   H*deltaU = rhs from the quadratic cost (Section 4):
%
%   J = sum_{i=1}^{Np} (x(k+i)-xTarget)' * Q * (x(k+i)-xTarget)
%     + sum_{j=0}^{Nc-1} delta_u(j)' * R * delta_u(j)
%     + sum_{j=0}^{Nc-1} (delta_u(j)-delta_u(j-1))' * S * (delta_u(j)-delta_u(j-1))
%
%   with delta_u(-1) := 0. (This controller is called fresh, stateless,
%   every control step per the required interface -- Section 9 -- so no
%   previous-command history is carried between calls; the increment
%   cost for the very first move in each new horizon is measured
%   against zero. See documentation/MPC_ENGINE.md.)
%
%   [H, rhs, diagInfo] = buildCostMatrices(F, Phi, x0, xTargetVec, Q, R, S, Np, Nc)
%
%   Q - 6x6 (or length-6 diagonal vector) state tracking weight
%   R - 2x2 (or length-2 diagonal vector) control effort weight
%   S - 2x2 (or length-2 diagonal vector) control-increment weight
%
%   Outputs H and rhs are meant to be solved with gaussJordanSolve.m
%   (Section 6) -- this function does NOT perform the solve itself and
%   does NOT use backslash or inv().
%
%   diagInfo carries Qbar/Rbar/Sbar/D/f0 for diagnostic reuse (e.g. cost
%   recomputation) by the caller, so they are not rebuilt twice.

    n = size(F,2);
    m = size(Phi,2) / Nc;

    Q = expandWeight(Q, n);
    R = expandWeight(R, m);
    S = expandWeight(S, m);

    Qbar = kron(eye(Np), Q);
    Rbar = kron(eye(Nc), R);

    % Difference operator D: (D*deltaU) stacks delta_u(j)-delta_u(j-1),
    % with delta_u(-1) = 0, for j = 0..Nc-1.
    D = zeros(m*Nc, m*Nc);
    D(1:m, 1:m) = eye(m);
    for j = 2:Nc
        rowsJ = (j-1)*m + (1:m);
        D(rowsJ, (j-1)*m + (1:m)) = eye(m);
        D(rowsJ, (j-2)*m + (1:m)) = D(rowsJ, (j-2)*m + (1:m)) - eye(m);
    end
    Sbar = kron(eye(Nc), S);

    xTargetStacked = repmat(xTargetVec, Np, 1);
    f0 = F*x0 - xTargetStacked;   % predicted-error offset if deltaU = 0

    H   = Phi.'*Qbar*Phi + Rbar + D.'*Sbar*D;
    rhs = -Phi.'*Qbar*f0;

    % Symmetrize to remove floating-point asymmetry before solving.
    H = (H + H.')/2;

    diagInfo = struct('Qbar', Qbar, 'Rbar', Rbar, 'Sbar', Sbar, 'D', D, 'f0', f0);
end

function W = expandWeight(w, n)
    if isvector(w) && ~isequal(size(w), [n n])
        W = diag(w(:));
    else
        W = w;
    end
    if ~isequal(size(W), [n n])
        error('buildCostMatrices:badWeight', ...
            'Weight matrix must be %dx%d or a length-%d vector.', n, n, n);
    end
end
