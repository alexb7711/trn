% Input:
%   x_buff: Nav state buffer
%   simpar: Simulation parameters
%
% Output
%   input_truth: Input values for truth state
%
function [input_nav] = inputNav(x_buff, ytilde_buff, t, i, q_moon, simpar)
    s                = extract_state(x_buff(:,i-1), simpar, 'nav');

    input_nav.Tib    = Ti2b(s.pos, s.vel, simpar);
    input_nav.a_meas = ytilde_buff(:, i-1);
    input_nav.n_a    = zeros(3,1);
    input_nav.simpar = simpar;
    input_nav.t      = t(i);
    input_nav.u      = zeros(3,1);
    input_nav.w      = zeros(3,1);
    input_nav.q_moon = q_moon;
end
