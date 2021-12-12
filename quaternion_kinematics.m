function [ qdot_i2b ] = quaternion_kinematics( omega_bi_b, q_i2b )
%quaternion_kinematics computes the rate of change of the quaternion
%
% Inputs:
%   omega_bi_b = angular rate of b wrt i expressed in b (rad/s)
%   q_i2b = quaternion from i to b
%
% Outputs
%   qdot_i2b = quaternion rate of change (units)
%
% Example Usage
% [ qdot_i2b ] = quaternion_kinematics( omega_bi_b, q_i2b )

% Author: Randy Christensen
% Date: 13-May-2020 11:35:13
% Reference: Rotations, Transformations, Left Quaternions, Right
% Quaternions?, Renato Zanetti, 2019, equation after equation 67
% Copyright 2020 Utah State University

qdot_i2b = 0.5*qmult([0;omega_bi_b],q_i2b);
end
