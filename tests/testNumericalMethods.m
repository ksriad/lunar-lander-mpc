function report = testNumericalMethods()
%TESTNUMERICALMETHODS Validation tests for all Person 3 numerical methods.
%
%   report = testNumericalMethods()
%
% Runs each hand-coded numerical method against a known
% analytical/reference case and records the expected value, computed
% value, absolute error, relative error, and iteration count (or other
% relevant extra info) where applicable. Prints a summary table and
% returns it as a MATLAB table.

    rows = {};

    % ==================== Experiment 3: Lagrange interpolation ====================
    xData = [0 1 2 3];
    yData = (xData - 1).^2;              % known quadratic (x-1)^2
    xq = 1.5;
    expected = (xq - 1)^2;
    computed = lagrangeInterpolation(xData, yData, xq);
    rows = addRow(rows, 'Lagrange interpolation: (x-1)^2 at x=1.5 (Exp.3)', expected, computed, NaN);

    % ==================== Experiment 4: linear regression ====================
    xr = (1:10)';
    yr = 3*xr + 2;                        % exact line y = 2 + 3x, no noise
    coeff = linearRegression(xr, yr);
    rows = addRow(rows, 'Linear regression intercept a0 (Exp.4)', 2, coeff(1), NaN);
    rows = addRow(rows, 'Linear regression slope a1 (Exp.4)',     3, coeff(2), NaN);

    % ==================== Experiment 4: polynomial regression ====================
    xp = (-5:5)';
    yp = 2*xp.^2 - 3*xp + 1;              % exact quadratic, no noise
    coeffP = polynomialRegression(xp, yp, 2);
    expP = [1; -3; 2];
    for i = 1:3
        rows = addRow(rows, sprintf('Polynomial regression coeff(%d) (Exp.4)', i), expP(i), coeffP(i), NaN);
    end

    % ==================== Experiment 5: Gauss-Jordan elimination ====================
    A = [2 1 -1; -3 -1 2; -2 1 2];
    b = [8; -11; -3];
    expX = [2; 3; -1];                    % known solution
    [xGJ, infoGJ] = gaussJordanElimination(A, b);
    for i = 1:3
        rows = addRow(rows, sprintf('Gauss-Jordan x(%d) (Exp.5)', i), expX(i), xGJ(i), infoGJ.residual);
    end

    % ==================== Experiment 6: finite difference on sin(x) ====================
    f = @sin; df = @cos; x0 = pi/4; h = 1e-4;
    computedFD = finiteDifference(f, x0, h, 'central');
    rows = addRow(rows, 'Central finite difference of sin(x) at pi/4 (Exp.6)', df(x0), computedFD, NaN);

    % ==================== Experiment 6: Richardson extrapolation ====================
    [dRich, errEst] = richardsonDerivative(f, x0, 1e-2);
    rows = addRow(rows, 'Richardson-extrapolated derivative of sin(x) at pi/4 (Exp.6)', df(x0), dRich, errEst);

    % ==================== Experiment 7: composite Trapezoidal ====================
    a = 0; b2 = pi; expectedInt = 2;      % integral of sin(x) from 0 to pi
    xT = linspace(a, b2, 101); yT = sin(xT);
    rows = addRow(rows, 'Composite Trapezoidal integral of sin(x), 0..pi (Exp.7)', ...
        expectedInt, compositeTrapezoidal(xT, yT), NaN);

    % ==================== Experiment 7: composite Simpson 1/3 ====================
    xS1 = linspace(a, b2, 101); yS1 = sin(xS1);   % 100 subintervals (even)
    rows = addRow(rows, 'Composite Simpson 1/3 integral of sin(x), 0..pi (Exp.7)', ...
        expectedInt, compositeSimpson13(xS1, yS1), NaN);

    % ==================== Experiment 7: composite Simpson 3/8 ====================
    xS2 = linspace(a, b2, 100); yS2 = sin(xS2);   % 99 subintervals (multiple of 3)
    rows = addRow(rows, 'Composite Simpson 3/8 integral of sin(x), 0..pi (Exp.7)', ...
        expectedInt, compositeSimpson38(xS2, yS2), NaN);

    % ==================== Experiment 7: adaptive integration ====================
    [Iadapt, infoAdapt] = adaptiveIntegration(@sin, a, b2, 1e-8);
    rows = addRow(rows, 'Adaptive Simpson integral of sin(x), 0..pi (Exp.7)', ...
        expectedInt, Iadapt, infoAdapt.evaluations);

    % ==================== Experiment 8: root finding ====================
    fRoot = @(x) x.^2 - 2; dfRoot = @(x) 2*x; expRoot = sqrt(2);   % known root sqrt(2)

    [r, info] = bisection(fRoot, 0, 2, 1e-10, 200);
    rows = addRow(rows, 'Bisection root of x^2-2 (Exp.8)', expRoot, r, info.iterations);

    [r, info] = falsePosition(fRoot, 0, 2, 1e-10, 200);
    rows = addRow(rows, 'False Position root of x^2-2 (Exp.8)', expRoot, r, info.iterations);

    [r, info] = newtonRaphson(fRoot, dfRoot, 1, 1e-12, 100);
    rows = addRow(rows, 'Newton-Raphson root of x^2-2 (Exp.8)', expRoot, r, info.iterations);

    [r, info] = secant(fRoot, 0, 2, 1e-12, 100);
    rows = addRow(rows, 'Secant root of x^2-2 (Exp.8)', expRoot, r, info.iterations);

    % ===============================================================================
    report = cell2table(rows, 'VariableNames', ...
        {'Test','Expected','Computed','AbsError','RelError','IterationsOrExtra'});
    disp(report);
end

function rows = addRow(rows, name, expected, computed, extra)
    absErr = abs(expected - computed);
    if expected ~= 0
        relErr = absErr / abs(expected);
    else
        relErr = NaN;
    end
    rows(end+1, :) = {name, expected, computed, absErr, relErr, extra};
end
