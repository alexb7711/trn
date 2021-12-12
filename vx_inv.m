function [ v ] = vx_inv( vcross )
%vx_inv returns the vector from a cross-product matrix
%
% Inputs:
%   vcross = 3x3xn cross product matrix
%
% Outputs
%   v = 3xn vector
%
% Example Usage
% [ v ] = vx_inv( vcross )

% Author: Randy Christensen
% Date: 03-Jun-2019 16:54:19
% Reference: Savage Strapdown Analytics, Eqn 3.1.1-14
% Copyright 2019 Utah State University

%% Validate inputs
[m,n,~] = size(vcross);
assert(m == 3 && n == 3,'Incorrect input size')
%% Compute vector
v = [squeeze(vcross(3,2,:))';
    squeeze(vcross(1,3,:))';
    squeeze(vcross(2,1,:))'];
end
