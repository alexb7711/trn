%estimate_error_state_vector calculates the estimate of the error state
%vector
%
% Inputs:
%   Input1 = description (units)
%   Input2 = description (units)
%
% Outputs
%   Output1 = description (units)
%   Output2 = description (units)
%
function [ del_x ] = estimate_error_state_vector(res,K)
    del_x = K*res;
end
