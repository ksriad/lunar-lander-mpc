function [root, info] = newtonRaphson(f, df, x0, tol, maxIter)
%NEWTONRAPHSON Hand-coded Newton-Raphson root-finding method.
%   EEE 212, Experiment 8 (Nonlinear equations).
%
%   [root, info] = newtonRaphson(f, df, x0, tol, maxIter)
%
% Inputs:
%   f       - function handle
%   df      - function handle for the derivative of f
%   x0      - initial guess
%   tol     - convergence tolerance on |x_{k+1} - x_k| (default 1e-8)
%   maxIter - maximum iterations (default 100)
%
% Does NOT call fzero.
%
% Outputs:
%   root - estimated root
%   info - struct with .iterations, .error (|f(root)|), .converged,
%          .history (root estimate at each iteration)

    if nargin < 4 || isempty(tol), tol = 1e-8; end
    if nargin < 5 || isempty(maxIter), maxIter = 100; end

    x = x0;
    history = zeros(maxIter, 1);
    converged = false;

    for k = 1:maxIter
        fx = f(x);
        dfx = df(x);

        if abs(dfx) < eps
            error('newtonRaphson:zeroDerivative', ...
                'Derivative is (numerically) zero at x = %g; Newton-Raphson cannot continue.', x);
        end

        xNew = x - fx / dfx;
        history(k) = xNew;

        if abs(xNew - x) < tol
            converged = true;
            history = history(1:k);
            root = xNew;
            info = makeInfo(k, abs(f(xNew)), converged, history);
            return;
        end
        x = xNew;
    end

    root = x;
    info = makeInfo(maxIter, abs(f(x)), converged, history);
end

function info = makeInfo(iterations, err, converged, history)
    info.iterations = iterations;
    info.error = err;
    info.converged = converged;
    info.history = history(:);
end
