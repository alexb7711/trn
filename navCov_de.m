%navCov_de computes the derivative of the nav state covariance
%
% Inputs:
%   Phat = nav state (mixed units)
%   input = input (mixed units)
%
% Outputs
%   Phat_dot = nav state derivative (mixed units)
%
function [ P_dot ] = navCov_de( P, input )
    % Unpack the inputs for clarity
    xhat   = input.xhat;
    simpar = input.simpar;

    % Compute state dynamics matrix
    F = calc_F(xhat, input);

    % Compute process noise coupling matrix
    G = calc_G(xhat, input);

    % Compute process noise PSD matrix
    Q = calc_Q(xhat, simpar);

    % Compute Phat_dot
    P_dot = F*P + P*F' + G*Q*G';
end
