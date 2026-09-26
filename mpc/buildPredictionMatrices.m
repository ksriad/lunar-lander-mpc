function [F, Phi] = buildPredictionMatrices(Ad, Bd, Np, Nc)
%BUILDPREDICTIONMATRICES Build the standard receding-horizon prediction
%   matrices so that the stacked predicted states over the horizon are:
%
%       X = F*x0 + Phi*deltaU
%
%   where
%       x0     : 6x1 current state
%       deltaU : (m*Nc)x1 stacked FUTURE CONTROL INCREMENTS
%                deltaU = [delta_u(0); delta_u(1); ...; delta_u(Nc-1)]
%       X      : (n*Np)x1 stacked predicted states
%                X = [x(k+1); x(k+2); ...; x(k+Np)]
%
%   Control is held constant at delta_u(Nc-1) for all steps beyond the
%   control horizon (standard control-horizon blocking, Nc <= Np).
%
%   [F, Phi] = buildPredictionMatrices(Ad, Bd, Np, Nc)
%
%   Inputs:
%       Ad, Bd - discrete-time model, x(k+1) = Ad*x(k) + Bd*delta_u(k)
%       Np     - prediction horizon (integer > 0)
%       Nc     - control horizon (integer, 0 < Nc <= Np)
%
%   Outputs:
%       F   - (n*Np) x n
%       Phi - (n*Np) x (m*Nc)
%
%   DERIVATION:
%       x(k+i) = Ad^i*x0 + sum_{j=0}^{i-1} Ad^(i-1-j)*Bd*u(j)
%   where u(j) = deltaU(j) for j < Nc, and u(j) = deltaU(Nc-1) (held)
%   for j >= Nc. Column c < Nc of the Phi block-row i is Ad^(i-c)*Bd
%   (only present once, when i>=c); column Nc absorbs every step from
%   Nc-1 up to i-1, since all of those steps apply the held value
%   deltaU(Nc-1).

    n = size(Ad,1);
    m = size(Bd,2);

    if size(Ad,2) ~= n
        error('buildPredictionMatrices:badAd', 'Ad must be square.');
    end
    if size(Bd,1) ~= n
        error('buildPredictionMatrices:badBd', 'Bd row count must match Ad.');
    end
    if Nc < 1 || Nc > Np
        error('buildPredictionMatrices:badHorizon', ...
            'Require 1 <= Nc <= Np (got Nc=%d, Np=%d).', Nc, Np);
    end

    % Precompute Ad^0 .. Ad^Np once (reused across all blocks).
    Apow = cell(Np+1,1);
    Apow{1} = eye(n);
    for p = 1:Np
        Apow{p+1} = Apow{p} * Ad;
    end

    F   = zeros(n*Np, n);
    Phi = zeros(n*Np, m*Nc);

    for i = 1:Np
        rows = (i-1)*n + (1:n);
        F(rows,:) = Apow{i+1};

        for c = 1:Nc
            cols = (c-1)*m + (1:m);
            if c < Nc
                if i >= c
                    Phi(rows, cols) = Apow{i-c+1} * Bd;
                end
                % else: control move c hasn't been applied by step i yet -> 0
            else % c == Nc: absorbs all steps j = Nc-1 .. i-1 (held control)
                if i >= Nc
                    blockSum = zeros(n,m);
                    for p = 0:(i-Nc)
                        blockSum = blockSum + Apow{p+1} * Bd;
                    end
                    Phi(rows, cols) = blockSum;
                end
            end
        end
    end

    % --- Dimension checks (Section 5 requirement) ---
    assert(isequal(size(F), [n*Np, n]), ...
        'buildPredictionMatrices:dimF', 'F has unexpected dimensions.');
    assert(isequal(size(Phi), [n*Np, m*Nc]), ...
        'buildPredictionMatrices:dimPhi', 'Phi has unexpected dimensions.');
end
