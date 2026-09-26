function I = compositeSimpson13(x, y)
%COMPOSITESIMPSON13 Hand-coded composite Simpson's 1/3 rule.
%   EEE 212, Experiment 7 (Numerical integration).
%
%   I = compositeSimpson13(x, y)
%
% Requires an EVEN number of subintervals (i.e. an ODD number of
% samples) and UNIFORM spacing, per the classic Simpson's 1/3 formula.
% Does NOT call trapz/integral/quadgk.
%
% To integrate a known function f over [a,b] with n (even) subintervals:
%   xs = linspace(a,b,n+1); ys = f(xs); I = compositeSimpson13(xs,ys);

    x = x(:); y = y(:);
    if numel(x) ~= numel(y)
        error('compositeSimpson13:sizeMismatch', 'x and y must be the same length.');
    end
    n = numel(x) - 1;
    if n < 2 || mod(n, 2) ~= 0
        error('compositeSimpson13:invalidN', ...
            'Simpson''s 1/3 rule requires an even number of subintervals (odd number of samples).');
    end

    h = (x(end) - x(1)) / n;
    spacing = diff(x);
    if any(abs(spacing - h) > 1e-9 * max(1, abs(h)))
        error('compositeSimpson13:nonuniform', ...
            'Simpson''s 1/3 rule requires uniformly spaced samples.');
    end

    I = y(1) + y(end);
    I = I + 4 * sum(y(2:2:end-1));
    I = I + 2 * sum(y(3:2:end-2));
    I = I * h / 3;
end
