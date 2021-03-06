%initialize_truth_state initialize the truth state vector consistent with
%the initial covariance matrix
%
% Inputs:
%   simpar : Simulation parameters
%
% Outputs:
%   x: Initialized truth state vector
%
% In the initialization of the truth and navigation states, you need only
% ensure that the estimation error is consistent with the initial
% covariance matrix.  One realistic way to do this is to set the true 
% vehicle states to the same thing every time, and randomize any sensor 
% parameters.
function [ x ] = initialize_truth_state(simpar)
    x = convert_struct_to_array(simpar.general.ic);
end
