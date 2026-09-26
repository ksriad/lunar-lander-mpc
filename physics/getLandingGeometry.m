function geom = getLandingGeometry(params)
%GETLANDINGGEOMETRY Single source of truth for the lander visual/contact geometry.
%   The simulation is planar (x-y with pitch theta). The three legs are the
%   2-D projection of a symmetric 120-degree three-leg arrangement around
%   the cylindrical fuselage: one rear leg projects to the centre and two
%   front legs project symmetrically left/right.
%
%   geom.feetLocal is N-by-2 [x y] in the lander's body frame. These are the
%   actual physical contact points used by the physics engine and landing test.

    if isfield(params, 'landerGeometry') && ~isempty(params.landerGeometry)
        g = params.landerGeometry;
    else
        g = struct();
    end

    geom.bodyWidth        = getOr(g, 'bodyWidth', 5.8);
    geom.bodyBottom      = getOr(g, 'bodyBottom', -2.8);
    geom.bodyTop         = getOr(g, 'bodyTop', 2.8);
    geom.noseHeight      = getOr(g, 'noseHeight', 2.0);
    geom.legAttachY      = getOr(g, 'legAttachY', -2.35);
    geom.legAttachX      = getOr(g, 'legAttachX', [-2.0 0 2.0]);
    geom.footX           = getOr(g, 'footX', [-3.4 0 3.4]);
    geom.footY           = getOr(g, 'footY', -5.3);
    geom.footPadHalfWidth = getOr(g, 'footPadHalfWidth', 0.75);
    geom.legThickness    = getOr(g, 'legThickness', 7.0);
    geom.footThickness   = getOr(g, 'footThickness', 8.0);

    geom.feetLocal = [geom.footX(:), repmat(geom.footY, numel(geom.footX), 1)];
    geom.attachLocal = [geom.legAttachX(:), repmat(geom.legAttachY, numel(geom.legAttachX), 1)];

    % The three foot centres span the same line at theta = 0. This is the
    % exact planar contact representation of the three physical pads.
    geom.footCount = size(geom.feetLocal, 1);
end

function value = getOr(s, field, default)
    if isfield(s, field) && ~isempty(s.(field))
        value = s.(field);
    else
        value = default;
    end
end
