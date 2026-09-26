function coeff = polynomialRegression(x, y, degree)
%POLYNOMIALREGRESSION Hand-coded least-squares polynomial regression.
%   EEE 212, Experiment 4 (Curve fitting).
%
%   coeff = polynomialRegression(x, y, degree)
%
% Fits yFit = coeff(1) + coeff(2)*x + ... + coeff(degree+1)*x^degree by
% forming and solving the least-squares normal equations with the
% hand-coded gaussJordanElimination function. Does NOT call polyfit
% (polyfit may only be used separately, as a validation reference).
%
% Note: normal equations become ill-conditioned at high degree; keep
% degree modest (this project uses it for things like thruster
% calibration curves, not high-order fits).

    x = x(:);
    y = y(:);
    if numel(x) ~= numel(y)
        error('polynomialRegression:sizeMismatch', 'x and y must be the same length.');
    end
    if degree < 1 || degree ~= floor(degree)
        error('polynomialRegression:invalidDegree', 'degree must be a positive integer.');
    end
    if numel(x) < degree + 1
        error('polynomialRegression:insufficientPoints', ...
            'At least degree+1 points are required.');
    end

    n = numel(x);
    X = ones(n, degree + 1);
    for p = 1:degree
        X(:, p + 1) = x .^ p;
    end

    normalA = X' * X;
    normalB = X' * y;

    coeff = gaussJordanElimination(normalA, normalB);
end
