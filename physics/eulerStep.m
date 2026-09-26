function stateNext = eulerStep(state, control, mass, params)
%EULERSTEP Explicit (forward) Euler integration of the lander dynamics.
%
%   stateNext = eulerStep(state, control, mass, params)
%
%   x_{k+1} = x_k + dt * f(x_k, u_k)
%
%   INPUTS: same as landerDynamics, plus params.dt (s), the fixed
%   simulation timestep (nominally 0.04 s per the project spec).
%
%   ASSUMPTION: control and mass are held constant (zero-order hold)
%   across the single sub-step. Mass changes by well under 0.1% of the
%   vehicle mass in one 0.04 s step for the parameter ranges used in this
%   project, so this is a negligible approximation - see
%   documentation/PHYSICS_ENGINE.md, "Mass update" for the justification.

    dstate    = landerDynamics(state, control, mass, params);
    stateNext = state + params.dt * dstate;

end
