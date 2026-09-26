function [root, info] = secant(f, x0, x1, tol, maxIter)
%SECANT Hand-coded Secant root-finding method.
%   EEE 212, Experiment 8 (Nonlinear equations).
%
%   [root, info] = secant(f, x0, x1, tol, maxIter)
%
% Does NOT call fzero.
%
% Outputs:
%   root - estimated root
%   info - struct with .iterations, .error (|f(root)|), .converged,
%          .history (root estimate at each iteration)

    if nargin < 4 || isempty(tol), tol = 1e-8; end
    if nargin < 5 || isempty(maxIter), maxIter = 100; end

    f0 = f(x0);
    f1 = f(x1);
    history = zeros(maxIter, 1);
    converged = false;

    for k = 1:maxIter
        if abs(f1 - f0) < eps
            error('secant:zeroDenominator', ...
                'f(x1) - f(x0) is (numerically) zero; secant method cannot continue.');
        end

        x2 = x1 - f1 * (x1 - x0) / (f1 - f0);
        history(k) = x2;
        f2 = f(x2);

        if abs(x2 - x1) < tol
            converged = true;
            history = history(1:k);
            root = x2;
            info = makeInfo(k, abs(f2), converged, history);
            return;
        end

        x0 = x1; f0 = f1;
        x1 = x2; f1 = f2;
    end

    root = x1;
    info = makeInfo(maxIter, abs(f1), converged, history);
end

function info = makeInfo(iterations, err, converged, history)
    info.iterations = iterations;
    info.error = err;
    info.converged = converged;
    info.history = history(:);
end
