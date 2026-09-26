function stateNext = rk2Step(state, control, mass, params)
%RK2STEP Second-order Runge-Kutta integration (explicit midpoint method).
%
%   stateNext = rk2Step(state, control, mass, params)
%
%       k1    = f(x_k, u_k)
%       x_mid = x_k + (dt/2) * k1
%       k2    = f(x_mid, u_k)
%       x_{k+1} = x_k + dt * k2
%
%   INPUTS: same as landerDynamics, plus params.dt (s).
%
%   ASSUMPTION: control and mass are held constant across the whole step,
%   including at the midpoint evaluation - consistent with eulerStep and
%   with the zero-order-hold assumption documented in
%   documentation/PHYSICS_ENGINE.md.

    dt = params.dt;

    k1       = landerDynamics(state, control, mass, params);
    midState = state + 0.5 * dt * k1;
    k2       = landerDynamics(midState, control, mass, params);

    stateNext = state + dt * k2;

end
