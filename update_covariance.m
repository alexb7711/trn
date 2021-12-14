%update_covariance updates the covariance matrix
%
% Inputs:
%   Input1 = description (units)
%   Input2 = description (units)
%
% Outputs
%   Output1 = description (units)
%   Output2 = description (units)
%
function [ cov_res ] = update_covariance( H,P,R )
    cov_res = H*P*H' + R;
end
