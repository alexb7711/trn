% Input:
%   x     : State vector
%   rad   : Radius
%   simpar: Simulation parameters
%
% Output:
%   g : Acceleration of gravity
%
function [g] = calc_grav(rad, simpar)
    mu  = simpar.general.MU;
    g   = -mu*rad/(norm(rad)^3);
end
