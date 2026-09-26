function plotData = prepareResponseData(telemetry, results, params)
%PREPARERESPONSEDATA Package processed data for Person 4's four required plots.
%
%   plotData = prepareResponseData(telemetry, results, params)
%
% This function does NOT draw anything and does NOT own any App
% Designer axes - it only prepares clean, ready-to-plot data for:
%   1) Position response : actual x(t) vs target x [vs predicted, if present]
%   2) Acceleration       : actual/numerical ax vs MPC/reference ax
%   3) Attitude           : theta vs reference/MPC theta
%   4) Thrust             : actual T vs 0 and Tmax
%
% Fields that depend on data Person 4/Person 2 may or may not provide
% (targetX, predictedX, referenceAx, predictedTheta, referenceTheta,
% params.Tmax) are left as [] when not present in telemetry/params, so
% Person 4 can check isempty(...) before plotting a given series.

    if nargin < 3
        params = struct();
    end

    t = telemetry.time(:);

    % 1) Position response ----------------------------------------------------
    plotData.position.time = t;
    plotData.position.actualX = telemetry.x(:);
    plotData.position.targetX = getFieldOrEmpty(telemetry, 'targetX', true);
    % predictedX may have a different (variable) length per MPC horizon,
    % so it is passed through as-is rather than forced to length(t).
    plotData.position.predictedX = getFieldOrEmpty(telemetry, 'predictedX', false);

    % 2) Acceleration -----------------------------------------------------------
    plotData.acceleration.time = t;
    plotData.acceleration.actualAx = telemetry.ax(:);
    plotData.acceleration.numericalAx = results.numericalAx;
    plotData.acceleration.referenceAx = getFieldOrEmpty(telemetry, 'referenceAx', true);

    % 3) Attitude -----------------------------------------------------------------
    plotData.attitude.time = t;
    plotData.attitude.theta = telemetry.theta(:);
    plotData.attitude.predictedTheta = getFieldOrEmpty(telemetry, 'predictedTheta', false);
    plotData.attitude.referenceTheta = getFieldOrEmpty(telemetry, 'referenceTheta', true);

    % 4) Thrust -----------------------------------------------------------------
    plotData.thrust.time = t;
    plotData.thrust.T = telemetry.T(:);
    plotData.thrust.zeroLine = zeros(size(t));
    if isfield(params, 'Tmax') && ~isempty(params.Tmax)
        plotData.thrust.TmaxLine = params.Tmax * ones(size(t));
    else
        plotData.thrust.TmaxLine = [];
    end
end

function val = getFieldOrEmpty(s, fieldName, forceColumn)
    if isfield(s, fieldName) && ~isempty(s.(fieldName))
        val = s.(fieldName);
        if forceColumn
            val = val(:);
        end
    else
        val = [];
    end
end
