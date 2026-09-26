function [x, info] = gaussJordanSolve(H, rhs)
%GAUSSJORDANSOLVE Hand-coded Gauss-Jordan elimination with partial
%   pivoting -- the required EEE 212 Experiment 5 mapping (Gauss-Jordan
%   elimination, pivoting, numerical stability, validation). This is the
%   core solve for the MPC normal equations and does NOT use MATLAB's
%   backslash (\) or inv() (Section 6/13).
%
%   [x, info] = gaussJordanSolve(H, rhs)
%
%   Solves H*x = rhs.
%
%   Inputs:
%       H   - n x n matrix (the MPC normal equations pass a symmetric
%             positive-(semi)definite H, but this solver does not
%             assume or require that structure)
%       rhs - n x 1 vector
%
%   Outputs:
%       x    - n x 1 solution (best-effort; check info.success)
%       info - struct:
%           .success        - true if solved without an unrecoverable
%                              singularity
%           .singular       - true if a zero pivot could not be resolved
%                              even after row swapping and a tiny ridge
%                              regularization
%           .illConditioned - true if the smallest pivot actually used
%                              was below a conditioning warning
%                              threshold relative to the matrix scale
%           .residual       - norm(H*x - rhs), computed even for
%                              degraded solutions, for diagnostics
%           .minPivot       - smallest |pivot| encountered
%           .regularized    - true if a tiny ridge term had to be added
%                              to proceed past a near-zero pivot

    n = size(H,1);
    if size(H,2) ~= n || numel(rhs) ~= n
        error('gaussJordanSolve:badSize', 'H must be n x n and rhs must have n elements.');
    end

    scale = max(1, norm(H, 'fro'));
    ZERO_PIVOT_TOL    = 1e-10 * scale;
    ILL_COND_WARN_TOL = 1e-8  * scale;
    RIDGE             = 1e-9  * scale;

    Aug = [H, rhs(:)];
    minPivot    = inf;
    singular    = false;
    regularized = false;

    for col = 1:n
        % --- partial pivoting: largest-magnitude entry at/below diagonal
        [pivotVal, relRow] = max(abs(Aug(col:n, col)));
        pivotRow = relRow + col - 1;

        if pivotVal < ZERO_PIVOT_TOL
            % Near-zero pivot even after searching for the best row:
            % regularize this single column with a tiny ridge term
            % rather than failing outright, and flag it clearly.
            Aug(col,col) = Aug(col,col) + RIDGE;
            regularized = true;
            pivotVal = abs(Aug(col,col));
            pivotRow = col;
            if pivotVal < ZERO_PIVOT_TOL
                singular = true;
            end
        end

        minPivot = min(minPivot, pivotVal);

        if pivotRow ~= col
            Aug([col pivotRow], :) = Aug([pivotRow col], :);
        end

        pivot = Aug(col,col);
        if pivot == 0
            singular = true;
            continue; % avoid divide-by-zero; this column stays degraded
        end
        Aug(col,:) = Aug(col,:) / pivot;

        for r = 1:n
            if r ~= col
                factor = Aug(r,col);
                if factor ~= 0
                    Aug(r,:) = Aug(r,:) - factor*Aug(col,:);
                end
            end
        end
    end

    x = Aug(:, end);

    info.residual       = norm(H*x - rhs);
    info.minPivot        = minPivot;
    info.singular        = singular;
    info.illConditioned    = minPivot < ILL_COND_WARN_TOL;
    info.regularized       = regularized;
    info.success           = ~singular && isfinite(info.residual);
end
