function [ q_a2b ] = rotv2quat( theta, n )
%rotv2quat computes a right-handed quaternion from an euler axis, n, and
%angle, theta, with the scalar (or real) term in the first component
%
% Inputs:
%   theta = angle of rotation (rad)
%   n = axis of rotation (unitless)
%
% Outputs
%   q_a2b = quaternion from the A frame to the B frame
%
% Example Usage
% [ q_a2b ] = rotv2quat( theta, n )s

% Author: Randy Christensen
% Date: 13-May-2020 09:02:39
% Reference: Rotations, Transformations, Left Quaternions, Right
% Quaternions?, Renato Zanetti, 2019, equation 30
% Copyright 2020 Utah State University

%% Validate inputs
[m_n,n_n] = size(n);
[m_th,n_th] = size(theta);
assert(m_n == 3,'Incorrect size of n')
assert(n_th == 1,'Incorrect size of theta')
assert(n_n == m_th,'Size of inputs must match')
%% Compute quaternion
q_a2b = [cos(theta'/2);sin(theta'/2).*n];
end
