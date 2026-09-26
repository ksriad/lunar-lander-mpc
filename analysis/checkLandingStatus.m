function [status, isLanded, isCrash] = checkLandingStatus(state, params)
%CHECKLANDINGSTATUS Classify the lander from the three physical foot pads.
%   SAFE touchdown requires all three pads to be simultaneously supported
%   by the landing platform, with the lander upright and within velocity /
%   angular-rate limits.  Contact is detected at the bottom of the legs,
%   never at the bottom of the fuselage.

    state = state(:);
    x = state(1);
    y = state(3);

    contact = getLandingContact(state, params);

    isLanded = false;
    isCrash  = false;

    tumbling = abs(state(5)) > pi/2;
    outOfBounds = abs(x) > 135 || y < params.target.y - 20;

    if tumbling || outOfBounds
        status = 'crash';
        isCrash = true;
        return
    end

    % A balanced three-pad contact is the only valid touchdown.
    if contact.contactDetected
        if contact.safeTouchdown
            status = 'safe';
            isLanded = true;
        else
            status = 'hard';
            isCrash = true;
        end
        return
    end

    status = 'flying';
end
