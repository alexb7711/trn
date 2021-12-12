% Input:
%   x     : Truth state vector
%   input : Simulation parameters
%
% Output:
%   a_thrust: Thrust acceleration of the vehicle
%
function [a_thrust] = calc_thrust(x, input)
    % Extract input variables
    simpar = input.simpar;
    n_a    = input.n_a;
    a_meas = input.a_meas;
    s      = extract_state(x, simpar, 'nav');

    % Calculate thrust
    a_thrust = a_meas - s.bias - n_a;
end
