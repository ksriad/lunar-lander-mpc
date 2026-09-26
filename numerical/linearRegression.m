function coeff = linearRegression(x, y)
%LINEARREGRESSION Hand-coded least-squares linear regression.
%   EEE 212, Experiment 4 (Curve fitting).
%
%   coeff = linearRegression(x, y)
%
% Fits yFit = coeff(1) + coeff(2)*x by forming and solving the
% least-squares normal equations with the hand-coded
% gaussJordanElimination function. Does NOT call polyfit.
%
% Output:
%   coeff = [a0; a1]   (intercept; slope)

    x = x(:);
    y = y(:);
    if numel(x) ~= numel(y)
        error('linearRegression:sizeMismatch', 'x and y must be the same length.');
    end
    if numel(x) < 2
        error('linearRegression:insufficientPoints', 'At least two points are required.');
    end

    X = [ones(size(x)), x];
    normalA = X' * X;
    normalB = X' * y;

    coeff = gaussJordanElimination(normalA, normalB);
end
