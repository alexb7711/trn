function [ delq ] = delq( theta )
%delq calculates the error vector
%
% Inputs:
%   theta = small angle rotation vectore (rad)
%
% Outputs
%   delq = error quaternion
%
% Example Usage
% [ delq ] = delq( theta )

% Author: Randy Christensen
% Date: 13-May-2020 15:36:08
% Reference: 
% Copyright 2020 Utah State University

%% Validate inputs
[m,n] = size(theta);
assert(m == 3 && n == 1,'Input must be a 3x1 vector');
%% Calculate error vector
delq = [1;theta/2];
end
