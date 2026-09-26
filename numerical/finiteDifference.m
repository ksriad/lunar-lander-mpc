function d = finiteDifference(f, x, h, method)
%FINITEDIFFERENCE Hand-coded finite-difference derivative estimate.
%   EEE 212, Experiment 6 (Numerical differentiation).
%
% TWO MODES:
%
% 1) Function-handle mode (used for validation against a known
%    analytical derivative, e.g. differentiating sin(x)):
%       d = finiteDifference(f, x, h, method)
%    where f is a function handle, x is a scalar/vector of evaluation
%    points, h is the step size, and method is 'forward' | 'backward' |
%    'central' (default 'central').
%
% 2) Discrete-data mode (used for telemetry, e.g. estimating velocity
%    from a position log, or acceleration from a velocity log):
%       d = finiteDifference(yData, xData)
%    where yData, xData are equal-length sample vectors (xData need not
%    be uniformly spaced). Central differences are used at interior
%    points; one-sided forward/backward differences are used at the
%    first/last sample, so the output is the same length as the input.
%
%    IMPORTANT: the result of mode (2) is a NUMERICALLY ESTIMATED
%    derivative computed from sampled telemetry. It is distinct from
%    any analytically known "model/physics" value the simulator's
%    physics engine may already carry (e.g. telemetry.ax/ay) - see
%    analysis/calculateFlightMetrics.m, which reports both and labels
%    them separately.

    if isa(f, 'function_handle')
        if nargin < 4 || isempty(method)
            method = 'central';
        end
        switch lower(method)
            case 'forward'
                d = (f(x + h) - f(x)) / h;
            case 'backward'
                d = (f(x) - f(x - h)) / h;
            case 'central'
                d = (f(x + h) - f(x - h)) / (2 * h);
            otherwise
                error('finiteDifference:invalidMethod', ...
                    'method must be ''forward'', ''backward'', or ''central''.');
        end
        return;
    end

    % ---- discrete-data (telemetry) mode ----
    yData = f(:);
    xData = x(:);
    if numel(yData) ~= numel(xData)
        error('finiteDifference:sizeMismatch', 'yData and xData must be the same length.');
    end
    n = numel(yData);
    if n < 2
        error('finiteDifference:insufficientPoints', 'At least two samples are required.');
    end

    d = zeros(n, 1);
    d(1) = (yData(2) - yData(1)) / (xData(2) - xData(1));       % forward at first sample
    d(n) = (yData(n) - yData(n-1)) / (xData(n) - xData(n-1));   % backward at last sample
    for k = 2:n-1
        d(k) = (yData(k+1) - yData(k-1)) / (xData(k+1) - xData(k-1)); % central, interior
    end

    d = reshape(d, size(x));
end
