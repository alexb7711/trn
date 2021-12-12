function [ qconj ] = qConjugate( q )
%qConjugate calculates the conjugate of the quaternion by negating the 
%vector portion, where the scalar (or real) term is the first component.
%
% Inputs:
%   q = 4xn input quaternions
%
% Outputs
%   qconj = 4xn ouputt quaternions
%
% Example Usage
% [ qconj ] = qConjugate( q )

% Author: Randy Christensen
% Date: 13-May-2020 08:52:26
% Reference: 
% Copyright 2020 Utah State University

%% Validate inputs
[m_q,~] = size(q);
assert(m_q == 4,'Input quaternion must be a 4xn matrix')
%% Compute quaternion conjugate
qconj = [q(1,:);
    -q(2:4,:)];
end