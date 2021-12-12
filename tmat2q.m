function [ q ] = tmat2q( T )
%tmat2q converts a transformation matrix to a quaternion where the scalar
%(or real) term in the first component
%
% Inputs:
%   T = tranformation matrix
%
% Outputs
%   q = quaternion (units)
%
% Example Usage
% [ q ] = tmat2q( T )

% Author: Randy Christensen
% Date: 01-Jun-2020 17:11:16
% Reference: Rotations, Transformations, Left Quaternions, Right
% Quaternions?, Renato Zanetti, 2019, equation 30
% Copyright 2020 Utah State University

q = dcm2quat(T)';
end