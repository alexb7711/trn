%compute_residual_example calculates the measurement residual
%
% Inputs:
%   s_meas: Synthesized measurement (expected measurement)
%   p_meas: Predicted measurement   (actual measurement)
%
% Outputs
%   res: Measurement residual
%
function [ res ] = compute_residual_example( s_meas, p_meas )
    % Residual is the difference between the measurement and the expected
    % measurement
    res = p_meas - s_meas;
end
