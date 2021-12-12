function [ dele ] = calcErrors( xhat, x, simpar )
%calcErrors computes estimation errors
%
% Inputs:
%   x_hat = estimated state vector(mixed units)
%   x = state vector (mixed units)
%
% Outputs
%   dele = estimation error state vector (mixed units)
%
% Example Usage
% [ dele ] = calcErrors( x_hat, x )

% Author: Randy Christensen
% Date: 21-May-2019 13:43:16
% Reference:
% Copyright 2019 Utah State University

%Get size of input and verify that it is a single vector
[~,m_x] = size(x);
[~, m_xhat] = size(xhat);
assert(m_x == m_xhat);
xhat_true = truth2nav(x);
dele = nan(simpar.states.nxfe,m_x);
%Calculate errors
for i=1:m_x
    dele = [];
end
end
