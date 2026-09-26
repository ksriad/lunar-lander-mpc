function I = compositeSimpson38(x, y)
%COMPOSITESIMPSON38 Hand-coded composite Simpson's 3/8 rule.
%   EEE 212, Experiment 7 (Numerical integration).
%
%   I = compositeSimpson38(x, y)
%
% Requires the number of subintervals to be a MULTIPLE OF 3, with
% UNIFORM spacing, per the classic Simpson's 3/8 formula. Does NOT call
% trapz/integral/quadgk.

    x = x(:); y = y(:);
    if numel(x) ~= numel(y)
        error('compositeSimpson38:sizeMismatch', 'x and y must be the same length.');
    end
    n = numel(x) - 1;
    if n < 3 || mod(n, 3) ~= 0
        error('compositeSimpson38:invalidN', ...
            'Simpson''s 3/8 rule requires the number of subintervals to be a multiple of 3.');
    end

    h = (x(end) - x(1)) / n;
    spacing = diff(x);
    if any(abs(spacing - h) > 1e-9 * max(1, abs(h)))
        error('compositeSimpson38:nonuniform', ...
            'Simpson''s 3/8 rule requires uniformly spaced samples.');
    end

    I = y(1) + y(end);
    for k = 2:n
        if mod(k - 1, 3) == 0
            I = I + 2 * y(k);
        else
            I = I + 3 * y(k);
        end
    end
    I = I * 3 * h / 8;
end
