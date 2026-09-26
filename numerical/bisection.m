function [root, info] = bisection(f, a, b, tol, maxIter)
%BISECTION Hand-coded bisection root-finding method.
%   EEE 212, Experiment 8 (Nonlinear equations).
%
%   [root, info] = bisection(f, a, b, tol, maxIter)
%
% Requires f(a) and f(b) to have opposite signs. Does NOT call fzero.
%
% Outputs:
%   root - estimated root
%   info - struct with .iterations, .error (final half-interval width),
%          .converged, .history (root estimate at each iteration)

    if nargin < 4 || isempty(tol), tol = 1e-6; end
    if nargin < 5 || isempty(maxIter), maxIter = 100; end

    fa = f(a);
    fb = f(b);
    if fa == 0
        root = a; info = makeInfo(0, 0, true, a); return;
    end
    if fb == 0
        root = b; info = makeInfo(0, 0, true, b); return;
    end
    if sign(fa) == sign(fb)
        error('bisection:noSignChange', 'f(a) and f(b) must have opposite signs.');
    end

    history = zeros(maxIter, 1);
    converged = false;
    c = (a + b) / 2;

    for k = 1:maxIter
        c = (a + b) / 2;
        fc = f(c);
        history(k) = c;

        if abs(fc) < eps || (b - a) / 2 < tol
            converged = true;
            history = history(1:k);
            root = c;
            info = makeInfo(k, (b - a) / 2, converged, history);
            return;
        end

        if sign(fc) == sign(fa)
            a = c; fa = fc;
        else
            b = c; fb = fc;
        end
    end

    root = c;
    info = makeInfo(maxIter, (b - a) / 2, converged, history);
end

function info = makeInfo(iterations, err, converged, history)
    info.iterations = iterations;
    info.error = err;
    info.converged = converged;
    info.history = history(:);
end
