function xhatdot = navState_de(xhat,input)
%navState_de computes the derivative of the nav state
%
% Inputs:
%   xhat = nav state (mixed units)
%   input = input (mixed units)
%
% Outputs
%   xhatdot = nav state derivative (mixed units)
%
% Example Usage
% xhatdot = navState_de(xhat,input)

% Author: Randy Christensen
% Date: 21-May-2019 10:40:10
% Reference: none
% Copyright 2019 Utah State University

%% Unpack the inputs
simpar = input.simpar;
omegatilde = input.ytilde;
%% Compute individual elements of x_dot
%% Assign to output
xhatdot = [];
end
