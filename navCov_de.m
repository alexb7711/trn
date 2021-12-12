function [ P_dot ] = navCov_de( P, input )
%navCov_de computes the derivative of the nav state covariance
%
% Inputs:
%   Phat = nav state (mixed units)
%   input = input (mixed units)
%
% Outputs
%   Phat_dot = nav state derivative (mixed units)
%
% Example Usage
% [ Phat_dot ] = navCov_de( Phat, input )

% Author: Randy Christensen
% Date: 21-May-2019 10:40:10
% Reference: none
% Copyright 2019 Utah State University

%Unpack the inputs for clarity
xhat = input.xhat;
simpar = input.simpar;
ytilde = input.ytilde;

%Compute state dynamics matrix
F = calc_F(xhat, ytilde, simpar);

%Compute process noise coupling matrix
G = calc_G(xhat, simpar);

%Compute process noise PSD matrix
Q = calc_Q(xhat, simpar);

%Compute Phat_dot
P_dot = [];
end
