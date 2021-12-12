function [ R ] = rotv2rotmat( theta, n )
%rotvtorotmat computes the rotation matrix which rotates the vector by an
%angle theta about the Euler axis n, expressed in the original coordinates
%
% Inputs:
%   theta = angle of rotation (rad)
%   n = axis of rotation (unitless)
%
% Outputs
%   R = rotation matrix
%
% Example Usage
% [ R ] = rotvtorotmat( theta, n )

% Author: Randy Christensen
% Date: 13-May-2020 08:38:11
% Reference: Rotations, Transformations, Left Quaternions, Right
% Quaternions?, Renato Zanetti, 2019, equation 3
% Copyright 2020 Utah State University

%% Validate inputs
[m_n,n_n] = size(n);
[m_th,n_th] = size(theta);
assert(m_n == 3,'Incorrect size of n')
assert(n_th == 1,'Incorrect size of theta')
assert(n_n == m_th,'Size of inputs must match')
%% Compute transformation matrix
%TODO: Vectorize these computations
R = zeros(3,3,n_n);
for i=1:n_n
    R(:,:,i) = eye(3) + sin(theta(i))*vx(n(:,i)) ...
        + (1-cos(theta(i)))*vx(n(:,i))*vx(n(:,i));
end
end