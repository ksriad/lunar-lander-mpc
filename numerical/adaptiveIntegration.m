function [I, info] = adaptiveIntegration(f, a, b, tol, maxDepth)
%ADAPTIVEINTEGRATION Hand-coded adaptive Simpson integration.
%   EEE 212, Experiment 7 (Numerical integration - adaptive).
%
%   [I, info] = adaptiveIntegration(f, a, b, tol, maxDepth)
%
% Recursively subdivides [a,b], comparing a single-panel Simpson
% estimate against the sum of two half-interval Simpson estimates. If
% they agree within tol, the (more accurate) refined estimate, with a
% Richardson-style correction, is accepted; otherwise the interval is
% bisected and each half is refined independently. Does NOT call
% integral/quadgk.
%
% Inputs:
%   f        - function handle
%   a, b     - integration limits
%   tol      - target absolute error for this call (default 1e-6)
%   maxDepth - maximum recursion depth (default 20)
%
% Outputs:
%   I    - estimated integral
%   info - struct with fields .evaluations (approx. count of Simpson
%          panel evaluations) and .maxDepthReached

    if nargin < 4 || isempty(tol), tol = 1e-6; end
    if nargin < 5 || isempty(maxDepth), maxDepth = 20; end

    info.evaluations = 0;
    info.maxDepthReached = 0;

    [I, info] = adaptiveSimpsonRecursive(f, a, b, tol, maxDepth, 0, info);
end

function [I, info] = adaptiveSimpsonRecursive(f, a, b, tol, maxDepth, depth, info)
    c = (a + b) / 2;
    Swhole = simpsonSinglePanel(f, a, b);
    Sleft  = simpsonSinglePanel(f, a, c);
    Sright = simpsonSinglePanel(f, c, b);
    info.evaluations = info.evaluations + 3;
    info.maxDepthReached = max(info.maxDepthReached, depth);

    if depth >= maxDepth || abs(Sleft + Sright - Swhole) < 15 * tol
        I = Sleft + Sright + (Sleft + Sright - Swhole) / 15;
    else
        [Il, info] = adaptiveSimpsonRecursive(f, a, c, tol/2, maxDepth, depth+1, info);
        [Ir, info] = adaptiveSimpsonRecursive(f, c, b, tol/2, maxDepth, depth+1, info);
        I = Il + Ir;
    end
end

function S = simpsonSinglePanel(f, a, b)
    c = (a + b) / 2;
    S = (b - a) / 6 * (f(a) + 4*f(c) + f(b));
end
