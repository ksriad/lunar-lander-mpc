function [xTerrain, yTerrain, xKnots, yKnots] = generateTerrain(params, xRange, nPoints)
%GENERATETERRAIN Build a smooth terrain profile using lagrangeInterpolation.
%   [xTerrain, yTerrain, xKnots, yKnots] = generateTerrain(params, xRange, nPoints)
%
%   xRange  : [xMin xMax] extent of the terrain to draw
%   nPoints : number of dense plotting points (default 200)
%
%   A small set of hand-placed knot points is interpolated with the
%   Lagrange routine to produce a rolling terrain that flattens out at
%   the landing pad (params.target.x, params.target.y).

    if nargin < 3 || isempty(nPoints)
        nPoints = 200;
    end

    padX = params.target.x;
    padY = params.target.y;
    padHW = params.target.halfWidth;

    % Knot points: rolling hills on either side, flat pad in the middle.
    xKnots = [xRange(1), padX-4*padHW, padX-1.5*padHW, padX-padHW, ...
              padX, padX+padHW, padX+1.5*padHW, padX+4*padHW, xRange(2)];
    yKnots = padY + [18, 10, 3, 0, 0, 0, 3, 9, 16];

    xTerrain = linspace(xRange(1), xRange(2), nPoints);
    yTerrain = lagrangeInterpolation(xKnots, yKnots, xTerrain);
end
