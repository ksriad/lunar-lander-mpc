function contact = getLandingContact(state, params)
%GETLANDINGCONTACT Evaluate the physical three-pad touchdown geometry.
%   The landing feet are the ONLY ground-contact points used by the
%   simulator.  The returned structure is shared by the physics engine,
%   landing-status logic, and GUI so the visible feet and the physics
%   cannot disagree about where touchdown occurs.
%
%   state = [x; vx; y; vy; theta; omega]

    state = state(:);
    if numel(state) ~= 6
        error('LandingContact:BadState', 'state must contain 6 elements.');
    end

    geom = getLandingGeometry(params);

    theta = state(5);
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    feetWorld = (R * geom.feetLocal')';
    feetWorld(:,1) = feetWorld(:,1) + state(1);
    feetWorld(:,2) = feetWorld(:,2) + state(3);

    padX  = params.target.x;
    padY  = params.target.y;
    padHW = params.target.halfWidth;

    footX = feetWorld(:,1);
    footY = feetWorld(:,2);

    contactTol = getLandingValue(params, 'footPenetrationTol', 0.02);
    heightTol  = getLandingValue(params, 'footHeightTol', 0.03);
    vxMax      = getLandingValue(params, 'vxMax', 0.8);
    vyMax      = getLandingValue(params, 'vyMax', 1.5);
    thetaMax   = getLandingValue(params, 'thetaMax', 0.005);
    omegaMax   = getLandingValue(params, 'omegaMax', 0.08);

    % Require the COMPLETE visible foot pad to remain on the landing pad,
    % not merely its centre point.
    usableHalfWidth = max(0, padHW - geom.footPadHalfWidth);
    allFeetInside = all(abs(footX - padX) <= usableHalfWidth + 1e-12);

    lowestFootY  = min(footY);
    highestFootY = max(footY);
    footHeightSpan = highestFootY - lowestFootY;

    surfaceContact = lowestFootY <= padY + contactTol;
    balanced = footHeightSpan <= heightTol && abs(theta) <= thetaMax;

    % For a simultaneous three-pad touchdown, all three foot centres must
    % be essentially on the same horizontal plane.  Requiring all three
    % to be no higher than the surface tolerance also prevents accepting
    % a single-leg impact as a touchdown.
    allFeetTouching = all(abs(footY - padY) <= contactTol);
    simultaneousContact = allFeetInside && allFeetTouching && balanced;

    contact.contactDetected   = surfaceContact;
    contact.allFeetInside     = allFeetInside;
    contact.allFeetTouching   = allFeetTouching;
    contact.balanced          = balanced;
    contact.simultaneous      = simultaneousContact;
    contact.lowestFootY       = lowestFootY;
    contact.highestFootY      = highestFootY;
    contact.footHeightSpan    = footHeightSpan;
    contact.feetWorld         = feetWorld;
    contact.footX             = footX;
    contact.footY             = footY;
    contact.altitude          = lowestFootY - padY;
    contact.usableHalfWidth   = usableHalfWidth;

    contact.safeVx    = abs(state(2)) <= vxMax;
    contact.safeVy    = abs(state(4)) <= vyMax;
    contact.safeTheta = abs(state(5)) <= thetaMax;
    contact.safeOmega = abs(state(6)) <= omegaMax;
    contact.safeKinematics = contact.safeVx && contact.safeVy && ...
                             contact.safeTheta && contact.safeOmega;

    contact.safeTouchdown = contact.simultaneous && contact.safeKinematics;
end

function value = getLandingValue(params, field, default)
    value = default;
    if isfield(params, 'landing') && isfield(params.landing, field) && ...
            ~isempty(params.landing.(field))
        value = params.landing.(field);
    end
end
