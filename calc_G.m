%calc_G Calculates the process noise dynamic coupling matrix
%
% Inputs:
%   xhat = state vector
%   simpar= simulation parameters
%
% Outputs
%   G = process noise dynamic coupling matrix
%
% Example Usage
% [ G ] = calc_G( xhat, simpar )

% Author: Randy Christensen
% Date: 13-May-2020
% Reference: None
% Copyright 2020 Utah State University
function [ G ] = calc_G( ~, input )

    %----------------------------------------------------------------------------
    % Unpack the inputs
    simpar = input.simpar;
    Tbi    = input.Tib';

    %----------------------------------------------------------------------------
    % Compute G
    G(1:3,:)   = [zeros(3), zeros(3), zeros(3), zeros(3)];
    G(4:6,:)   = [-Tbi, eye(3), zeros(3), zeros(3)];
    G(7:9,:)   = [zeros(3), zeros(3), eye(3), zeros(3)];
    G(10:12,:) = [zeros(3), zeros(3), zeros(3), eye(3)];
    G(13:15,:) = [zeros(3), zeros(3), zeros(3), zeros(3)];
    G(16:18,:) = [zeros(3), zeros(3), zeros(3), zeros(3)];
    G(19:21,:) = [zeros(3), zeros(3), zeros(3), zeros(3)];
end
