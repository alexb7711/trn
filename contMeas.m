%contInertialMeas synthesizes noise measurements used to propagate the
%navigation state
%
% Inputs:
%   x    : Truth state vector
%   input: Structure of parameters used for continuous measurement
%
% Outputs
%   a_meas: Continuous measurement
%
function [ a_meas ] = contMeas(x, input)
    %%---------------------------------------------------------------------------
    % Extract input parameters
    Tib    = input.Tib;
    n_a    = input.n_a;
    simpar = input.simpar;

    %%---------------------------------------------------------------------------
    % Convert state into structure
    s = extract_state(x, simpar, 'truth');

    %%---------------------------------------------------------------------------
    % Calcualte gravity
    a_grav = calc_grav(simpar.general.R_M, simpar);

    %%---------------------------------------------------------------------------
    % Calculate thrust

    % TODO: Consider passsing through as input parameter and calculate thrust
    %       in runsim.m
    accel_t = Tib*(simpar.general.n*a_grav*Tib(1,:)');

    % Calculate thrust measurement
    a_meas  = accel_t + s.bias + n_a;
end
