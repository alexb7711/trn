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
% function [ a_meas ] = contMeas(x, input)
function [ a_meas ] = contMeas(x_new, x_old, input)
    simpar = input.simpar;
    s_old  = extract_state(x_old, simpar, 'nav');
    s_new  = extract_state(x_new, simpar, 'nav');

    mu      = simpar.general.MU;
    old_pos = s_old.pos;
    a_grav  = calc_grav(s_old.pos, simpar);

    v_old   = s_old.vel;
    v_new   = s_new.vel;
    delta_t = simpar.general.dt;

    a_meas = (v_new - v_old)/delta_t - a_grav + s_old.bias;
end
