% Input:
%   x_buff: Nav state buffer
%   simpar: Simulation parameters
%
% Output
%   input_truth: Input values for truth state
%
function [input_nav] = inputNav(a_meas, t, i, Tib, simpar)
    input_nav.Tib    = Tib;
    input_nav.a_meas = a_meas;
    input_nav.simpar = simpar;
    input_nav.t      = t(i);
    input_nav.u      = zeros(3,1);
    input_nav.w      = zeros(3,1);
end
