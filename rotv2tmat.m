function [ T_a2b ] = rotv2tmat( theta, n )
%rotv2dcm computes the transformation matrix from the A frame to the B
%frame by starting at the A frame, and rotating about the Euler axis, n, by 
%an angle theta
%
% Inputs:
%   theta = angle of rotation (rad)
%   n = axis of rotation (unitless)
%
% Outputs
%   T_a2b = tranformation matrix from the A frame to the B frame
%
% Example Usage
% [ T_a2b ] = rotv2dcm( theta, n )

% Author: Randy Christensen
% Date: 13-May-2020 08:38:11
% Reference: Rotations, Transformations, Left Quaternions, Right
% Quaternions?, Renato Zanetti, 2019, equation 11
% Copyright 2020 Utah State University

%% Validate inputs
[m_n,n_n] = size(n);
[m_th,n_th] = size(theta);
assert(m_n == 3,'Incorrect size of n')
assert(n_th == 1,'Incorrect size of theta')
assert(n_n == m_th,'Size of inputs must match')
%% Compute transformation matrix
%TODO: Vectorize these computations
T_a2b = zeros(3,3,n_n);
for i=1:n_n
    T_a2b(:,:,i) = eye(3) - sin(theta(i))*vx(n(:,i)) ...
        + (1-cos(theta(i)))*vx(n(:,i))*vx(n(:,i));
end
end