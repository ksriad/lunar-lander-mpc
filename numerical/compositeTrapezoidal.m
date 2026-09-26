function I = compositeTrapezoidal(x, y)
%COMPOSITETRAPEZOIDAL Hand-coded composite Trapezoidal rule.
%   EEE 212, Experiment 7 (Numerical integration).
%
%   I = compositeTrapezoidal(x, y)
%
% Integrates sampled data y(x) using the composite Trapezoidal rule.
% x need not be uniformly spaced. Does NOT call trapz.
%
% To integrate a known function f over [a,b] with n subintervals
% (used for validation against a known result), sample it first:
%   xs = linspace(a,b,n+1); ys = f(xs); I = compositeTrapezoidal(xs,ys);
%
% Project usage: integrating speed = sqrt(vx.^2+vy.^2) over telemetry
% time to obtain total flight path length (see
% analysis/calculateFlightMetrics.m), and optionally integrating
% mass-flow-rate T/ve for a numerical fuel-consumption cross-check.

    x = x(:); y = y(:);
    if numel(x) ~= numel(y)
        error('compositeTrapezoidal:sizeMismatch', 'x and y must be the same length.');
    end
    n = numel(x);
    if n < 2
        error('compositeTrapezoidal:insufficientPoints', 'At least two samples are required.');
    end

    I = 0;
    for k = 1:n-1
        h = x(k+1) - x(k);
        I = I + h * (y(k) + y(k+1)) / 2;
    end
end
