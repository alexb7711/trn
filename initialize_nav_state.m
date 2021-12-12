%initialize_nav_state initializes the navigation state vector consistent
%with the initial covariance matrix
%
% Inputs:
%   x      : Truth state vector
%   P      : Covariance matrix
%   simpar : Simulation parameters
%
% Outputs
%   xhat: Navigation state vector
%
% Consistent with the truth state initialization, you should randomize the
% vehicle states, and initialize any sensor parameters to zero.  An example
% of these calculations are shown below.
% function [ xhat ] = initialize_nav_state(x, P, simpar)
function [ xhat ] = initialize_nav_state(x, simpar)
% If injectErrors is disabled, read initial conditions from the configuration
% file
if ~simpar.sim.injectErrors
    xhat = truth2nav(x, simpar);
% Otherwise randomly generate a nav state vector
else
    % Initialize the nav state to a 'white' value
    [L_posvelatt,p] = chol(P(simpar.states.ixfe.vehicle, simpar.states.ixfe.vehicle,1), 'lower');

    assert(p == 0, 'Phat_0 is not positive definite');

    delx_0                               = zeros(simpar.states.nxfe,1);
    delx_0(simpar.states.ixfe.vehicle,1) = L_posvelatt*randn(length(simpar.states.ixfe.vehicle),1);

    xhat                                 = injectErrors(truth2nav(x),delx_0, simpar);
    xhat(simpar.states.ixf.parameter,1)  = 0;
end % if

end
