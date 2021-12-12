function [ x_hat_c ] = correctErrors( x_hat, dele, simpar)
%correctState corrects the state vector given an estimated error state
%vector
%
% Inputs:
%   x_hat = estimated state vector (mixed units)
%   delx = error state vector (mixed units)
%
% Outputs
%   x = description (units)
%
% Example Usage
% [ x ] = correctState( x_hat, delx )

% Author: Randy Christensen
% Date: 10-Dec-2018 11:44:28
% Reference:
% Copyright 2018 Utah State University

%Get size of input and verify that it is a single vector
[~,m_x] = size(x_hat);
[~, m_delx] = size(dele);
assert(m_x == m_delx);
x_hat_c = nan(simpar.states.nxf,m_x);
%Correct errors
for i=1:m_x
    x_hat_c = [];  
end
end
