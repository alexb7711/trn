function [ K ] = compute_Kalman_gain( H,P,resCov )
%compute_Kalman_gain calculates the Kalman gain
%
% Inputs:
%   Input1 = description (units)
%   Input2 = description (units)
%
% Outputs
%   Output1 = description (units)
%   Output2 = description (units)
%
% Example Usage
% [ output_args ] = compute_Kalman_gain( input_args )
%
% See also FUNC1, FUNC2

% Author: Randy Christensen
% Date: 31-Aug-2020 16:01:57
% Reference: 
% Copyright 2020 Utah State University
K = P*H'*inv(resCov);
end
