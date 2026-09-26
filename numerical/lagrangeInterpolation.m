function yQuery = lagrangeInterpolation(xData, yData, xQuery)
%LAGRANGEINTERPOLATION Hand-coded Lagrange polynomial interpolation.
%   EEE 212, Experiment 3 (Interpolation).
%
%   yQuery = lagrangeInterpolation(xData, yData, xQuery)
%
% Inputs:
%   xData  - vector of known x-coordinates (must be distinct)
%   yData  - vector of known y-coordinates, same length as xData
%   xQuery - scalar or vector of query points at which to interpolate
%
% Output:
%   yQuery - interpolated value(s), same size as xQuery
%
% Notes:
%   - This is a direct implementation of the Lagrange interpolation
%     formula. It does NOT call interp1.
%   - Intended for small/local point sets. High-degree interpolation
%     over large datasets is avoided elsewhere in this project because
%     of Runge's-phenomenon-style oscillation and conditioning issues;
%     callers should pass a small, local subset of points (e.g. nearby
%     terrain samples) rather than an entire large dataset.
%
% Project usage: lunar terrain / landing-pad elevation interpolation,
% using a small local set of terrain samples around the query location.

    xData = xData(:)';
    yData = yData(:)';

    if numel(xData) ~= numel(yData)
        error('lagrangeInterpolation:sizeMismatch', ...
            'xData and yData must have the same number of elements.');
    end

    n = numel(xData);
    if n < 2
        error('lagrangeInterpolation:insufficientPoints', ...
            'At least two data points are required.');
    end

    if numel(unique(xData)) ~= n
        error('lagrangeInterpolation:duplicateX', ...
            'xData contains duplicate x values; Lagrange interpolation requires distinct nodes.');
    end

    origSize = size(xQuery);
    xQueryCol = xQuery(:);
    yQuery = zeros(size(xQueryCol));

    for q = 1:numel(xQueryCol)
        xq = xQueryCol(q);
        total = 0;
        for i = 1:n
            Li = 1;
            for j = 1:n
                if j ~= i
                    Li = Li * (xq - xData(j)) / (xData(i) - xData(j));
                end
            end
            total = total + yData(i) * Li;
        end
        yQuery(q) = total;
    end

    yQuery = reshape(yQuery, origSize);
end
