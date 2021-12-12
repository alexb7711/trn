% Input:
%   x_buff: Truth state buffer
%   simpar: Simulation parameters
%
% Output
%   input_truth: Input values for truth state
%
function [input_truth] = inputTruth(x_buff, ytilde_buff, t, i, simpar)
    s                  = extract_state(x_buff(:,i-1), simpar, 'truth');

    input_truth.Tib    = Ti2b(s.pos, s.vel, simpar);
    input_truth.a_meas = ytilde_buff(:, i-1);
    input_truth.n_a    = zeros(3,1);
    input_truth.simpar = simpar;
    input_truth.t      = t(i);
    input_truth.thrust = calc_thrust(x_buff(:,i-1), input_truth);
    input_truth.u      = zeros(3,1);
    input_truth.w      = zeros(3,1);
end
