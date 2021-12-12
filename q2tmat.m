function [ T_a2b ] = q2tmat( q_a2b )
%q2dcm computes a transformation matrix from a quaternion where the scalar
%(or real) term in the first component
%
% Inputs:
%   q_a2b = quaternion
%
% Outputs
%   T_a2b = transformation from the A frame to the B frame
%
% Example Usage
% [ T_a2b ] = q2dcm( q_a2b )

% Author: Randy Christensen
% Date: 13-May-2020 08:52:26
% Reference: Rotations, Transformations, Left Quaternions, Right
% Quaternions?, Renato Zanetti, 2019, equation 39
% Copyright 2020 Utah State University

%% Validate inputs
[m_q,n_q] = size(q_a2b);
assert(m_q == 4,'Input quaternion must be a 4xn matrix')
%% Compute transformation matrix
%TODO: Vectorize these calculationss
q_scalar = q_a2b(1,:);
q_vec = q_a2b(2:4,:);
T_a2b = zeros(3,3,n_q);
for i=1:n_q
    T_a2b(:,:,i) = eye(3,3) - 2*q_scalar(i)*vx(q_vec(:,i)) ...
        + 2*vx(q_vec(:,i))*vx(q_vec(:,i));
end
end