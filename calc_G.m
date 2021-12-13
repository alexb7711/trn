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
    G = zeros(12, 21);
    G(simpar.states.ix.vel      , simpar.states.ix.pos)      = -input.Tbi;
    G(simpar.states.ix.vel      , simpar.states.ix.vel)      = eye(3);
    G(simpar.states.ix.cam      , simpar.states.ix.cam)      = eye(3);
    G(simpar.states.ix.acc_bias , simpar.states.ix.acc_bias) = eye(3);
end
