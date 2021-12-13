%calc_F computes the dynamics coupling matrix
%
% Inputs:
%   xhat = state vector
%   ytilde = continuous measurements
%   simpar = simulation parameters
%
% Outputs
%   Fhat = state dynamics matrix
%
function [ Fhat ] = calc_F(xhat, input)
    %----------------------------------------------------------------------------
    % Unpack the inputs
    simpar = input.simpar;
    Tbi    = input.Tib';
    s      = extract_state(xhat, simpar, 'nav');
    ur     = s.pos/norm(s.pos);

    %----------------------------------------------------------------------------
    % Compute Fhat
    Fhat = zeros(simpar.states.nxf);

    Fhat(simpar.states.ix.pos      , simpar.states.ix.vel)      = eye(3);
    Fhat(simpar.states.ix.vel      , simpar.states.ix.pos)      = -simpar.general.MU/(norm(s.pos)^3) * (eye(3) - 3*ur*ur');
    Fhat(simpar.states.ix.vel      , simpar.states.ix.acc_bias) = -Tbi;
    Fhat(simpar.states.ix.cam      , simpar.states.ix.cam)      = -eye(3)/simpar.general.tau_c;
    Fhat(simpar.states.ix.acc_bias , simpar.states.ix.acc_bias) = -eye(3)/simpar.general.tau_b;
end
