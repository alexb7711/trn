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
function [ K ] = compute_Kalman_gain(H,P,resCov)
    K = P*H'*inv(resCov);
end
