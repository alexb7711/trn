% Input:
%   x     : Truth state vector
%   input : Simulation parameters
%
% Output:
%   a_thrust: Thrust acceleration of the vehicle
%
function [a_thrust] = calc_thrust(x, input)
    %%---------------------------------------------------------------------------
    % Extract input parameters
    Tib    = input.Tib;
    n_a    = input.n_a;
    simpar = input.simpar;

    %%---------------------------------------------------------------------------
    % Convert state into structure
    s = extract_state(x, simpar, 'truth');

    if input.t < input.simpar.general.thrust_disable
        %%-----------------------------------------------------------------------
        % Calcualte gravity
        a_grav = -calc_grav(simpar.general.R_M, simpar);

        %%-----------------------------------------------------------------------
        % Calculate thrust
        a_thrust = Tib*(simpar.general.n*a_grav*Tib(1,:)');
    else
        a_thrust  = zeros(3,1);
    end
end
