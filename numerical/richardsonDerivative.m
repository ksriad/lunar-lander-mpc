function [d, err] = richardsonDerivative(f, x, h, method)
%RICHARDSONDERIVATIVE Richardson-extrapolated derivative estimate.
%   EEE 212, Experiment 6 (Numerical differentiation).
%
%   [d, err] = richardsonDerivative(f, x, h, method)
%
% Combines two central-difference estimates (step h and step h/2) via
% Richardson extrapolation to cancel the leading O(h^2) error term of
% the central-difference formula, giving an O(h^4)-accurate estimate.
%
% The central-difference formula has error expansion
%   D(h) = f'(x) + c*h^2 + O(h^4)
% so halving h and combining via
%   D = (4*D(h/2) - D(h)) / 3
% cancels the h^2 term (the general Richardson rule for a leading-order
% O(h^p) estimate is D = (2^p*D(h/2) - D(h)) / (2^p - 1); here p=2, so
% 2^p=4, giving the 4/3 combination - NOT 16/15, which is the
% combination used when doubling the order a SECOND time, e.g. in
% Romberg integration's second column).
%
% Inputs:
%   f      - function handle, f(x)
%   x      - evaluation point (scalar or vector)
%   h      - base step size (default 1e-2)
%   method - accepted for interface symmetry with finiteDifference;
%            Richardson extrapolation here is built on the central
%            difference stencil, as in the lab methodology.
%
% Outputs:
%   d   - Richardson-extrapolated derivative estimate
%   err - estimated error, |D(h/2) - D(h)| / 3

    if nargin < 3 || isempty(h)
        h = 1e-2;
    end
    if nargin < 4
        method = 'central'; %#ok<NASGU>
    end

    Dh  = finiteDifference(f, x, h,   'central');
    Dh2 = finiteDifference(f, x, h/2, 'central');

    % D = (4*D(h/2) - D(h)) / 3  (Richardson extrapolation of an O(h^2) formula)
    d   = (4 * Dh2 - Dh) / 3;
    err = abs(Dh2 - Dh) / 3;
end
