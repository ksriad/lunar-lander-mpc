function [x, info] = gaussJordanElimination(A, b)
%GAUSSJORDANELIMINATION Hand-coded Gauss-Jordan elimination with partial pivoting.
%   EEE 212, Experiment 5 (Simultaneous linear algebraic equations).
%
%   [x, info] = gaussJordanElimination(A, b)
%
% TEAM INTEGRATION NOTE:
%   Person 2 owns the Gauss-Jordan solver used inside the MPC
%   controller. This function is Person 3's independently-tested
%   educational implementation, kept so that Experiment 5 can be
%   demonstrated and validated (see tests/testNumericalMethods.m) and
%   so that linearRegression.m / polynomialRegression.m have a
%   hand-coded solver to call for their normal equations. This is NOT
%   intended to be a second, competing solver inside the MPC loop -
%   at integration time this should be reconciled with Person 2's
%   shared solver so the shipped project uses one MPC solver only.
%
% Inputs:
%   A - n x n coefficient matrix
%   b - n x 1 right-hand-side vector
%
% Outputs:
%   x    - solution vector
%   info - struct with fields:
%            .residual      = norm(A*x - b)
%            .singular      = true if a (numerically) zero pivot was hit
%            .pivotSequence = row indices used as pivots, in column order
%
% Does NOT use A\b or inv(A).

    [n, m] = size(A);
    if n ~= m
        error('gaussJordanElimination:notSquare', 'A must be a square matrix.');
    end
    b = b(:);
    if numel(b) ~= n
        error('gaussJordanElimination:sizeMismatch', 'b must have the same number of rows as A.');
    end

    Aug = [A, b];
    pivotSequence = zeros(1, n);
    singular = false;
    tolPivot = 1e-12;

    for col = 1:n
        % Partial pivoting: choose the largest-magnitude entry at/below
        % the diagonal in this column as the pivot row.
        [maxVal, maxIdx] = max(abs(Aug(col:n, col)));
        pivotRow = maxIdx + col - 1;

        if maxVal < tolPivot
            singular = true;
            pivotSequence(col) = NaN;
            continue;
        end

        if pivotRow ~= col
            Aug([col, pivotRow], :) = Aug([pivotRow, col], :);
        end
        pivotSequence(col) = pivotRow;

        Aug(col, :) = Aug(col, :) / Aug(col, col);

        for row = 1:n
            if row ~= col
                factor = Aug(row, col);
                Aug(row, :) = Aug(row, :) - factor * Aug(col, :);
            end
        end
    end

    x = Aug(:, end);

    info.residual = norm(A * x - b);
    info.singular = singular;
    info.pivotSequence = pivotSequence;
end
