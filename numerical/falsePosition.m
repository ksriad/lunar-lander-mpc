function [root, info] = falsePosition(f, a, b, tol, maxIter)
%FALSEPOSITION Hand-coded False Position (Regula Falsi) root-finding method.
%   EEE 212, Experiment 8 (Nonlinear equations).
%
%   [root, info] = falsePosition(f, a, b, tol, maxIter)
%
% Requires f(a) and f(b) to have opposite signs. Does NOT call fzero.
%
% Outputs:
%   root - estimated root
%   info - struct with .iterations, .error (|f(root)| at convergence),
%          .converged, .history (root estimate at each iteration)

    if nargin < 4 || isempty(tol), tol = 1e-6; end
    if nargin < 5 || isempty(maxIter), maxIter = 200; end

    fa = f(a);
    fb = f(b);
    if sign(fa) == sign(fb)
        error('falsePosition:noSignChange', 'f(a) and f(b) must have opposite signs.');
    end

    history = zeros(maxIter, 1);
    converged = false;
    cPrev = a;
    c = a;

    for k = 1:maxIter
        c = b - fb * (b - a) / (fb - fa);
        fc = f(c);
        history(k) = c;

        if abs(fc) < tol || abs(c - cPrev) < tol
            converged = true;
            history = history(1:k);
            root = c;
            info = makeInfo(k, abs(fc), converged, history);
            return;
        end

        if sign(fc) == sign(fa)
            a = c; fa = fc;
        else
            b = c; fb = fc;
        end
        cPrev = c;
    end

    root = c;
    info = makeInfo(maxIter, abs(f(c)), converged, history);
end

function info = makeInfo(iterations, err, converged, history)
    info.iterations = iterations;
    info.error = err;
    info.converged = converged;
    info.history = history(:);
end
