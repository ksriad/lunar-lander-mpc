function [A, B, u0] = linearizeLander(m, I, g)
%LINEARIZELANDER Continuous-time linearization of the lander dynamics
%   about the hover operating point (theta = 0, T = m*g, tauRCS = 0).
%
%   [A, B, u0] = linearizeLander(m, I, g)
%
%   Inputs:
%       m  - current lander mass (kg), scalar, > 0
%       I  - moment of inertia about the lander's rotation axis (kg*m^2)
%       g  - gravitational acceleration (m/s^2), e.g. 1.62 for the Moon
%
%   Outputs:
%       A  - 6x6 continuous-time state Jacobian, evaluated at the hover
%            operating point.
%       B  - 6x2 continuous-time input Jacobian, evaluated at the SAME
%            operating point. Column 1 multiplies THRUST DEVIATION
%            (see note below), column 2 multiplies tauRCS.
%       u0 - 2x1 nominal control at the operating point:
%               u0 = [T0; tau0] = [m*g; 0]
%            Absolute control is reconstructed as u = u0 + delta_u.
%
%   STATE ORDER (shared project contract):
%       x = [x; vx; y; vy; theta; omega]
%   CONTROL ORDER:
%       u = [T; tauRCS]
%
%   DERIVATION
%   ----------
%   Nonlinear dynamics:
%       dx/dt     = vx
%       dvx/dt    = (T/m)*sin(theta)
%       dy/dt     = vy
%       dvy/dt    = (T/m)*cos(theta) - g
%       dtheta/dt = omega
%       domega/dt = tauRCS/I
%
%   Linearizing about theta0 = 0, omega0 = 0, T0 = m*g, tau0 = 0:
%
%       d(dvx/dt)/dtheta |_(0,T0) = (T0/m)*cos(0)  = g
%       d(dvx/dt)/dT     |_(0,T0) = sin(0)/m       = 0
%       d(dvy/dt)/dtheta |_(0,T0) = -(T0/m)*sin(0) = 0
%       d(dvy/dt)/dT     |_(0,T0) = cos(0)/m       = 1/m
%       d(domega/dt)/dtauRCS      = 1/I
%
%   IMPORTANT MODELING NOTE (avoids an affine/offset term):
%   dx/dt = vx and dy/dt = vy are ALREADY exactly linear (no
%   approximation needed), and the hover point satisfies
%   f(x_eq, u0) = 0 for ANY position/velocity with vx=vy=theta=omega=0.
%   Consequently position does not appear in A or B at all (translation
%   invariance: columns 1 and 3 of A are zero). This means the discrete
%   prediction model can be written directly in terms of the ABSOLUTE
%   state x and the control INCREMENT delta_u, with NO extra affine
%   term:
%
%       x_dot = A*x + B*delta_u          (delta_u = u - u0)
%
%   Full reasoning is documented in documentation/MPC_ENGINE.md. Do NOT
%   feed absolute u into Ad/Bd; B here is only valid multiplied by
%   delta_u, and u must be reconstructed afterward as u0 + delta_u.

    A = zeros(6,6);
    A(1,2) = 1;         % dx/dt = vx
    A(2,5) = g;         % dvx/dt sensitivity to theta at hover (T0/m = g)
    A(3,4) = 1;         % dy/dt = vy
    A(5,6) = 1;         % dtheta/dt = omega
    % A(4,:) and A(6,:) are zero: dvy/dt and domega/dt have no state
    % feedback terms at the hover linearization point.

    B = zeros(6,2);
    B(4,1) = 1/m;       % dvy/dt sensitivity to thrust deviation
    B(6,2) = 1/I;       % domega/dt sensitivity to torque

    u0 = [m*g; 0];
end
