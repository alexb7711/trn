%calc_Q Calculates the process noise power spectral density
%
% Inputs:
%   xhat = state vector
%   simpar= simulation parameters
%
% Outputs
%   Q = process noise dynamic coupling matrix
%
function [ Q ] = calc_Q(~, simpar)
    % Unpack the inputs
    Q_n = simpar.nav.params.vrw^2 * eye(3);
    Q_g = simpar.nav.params.Q_grav*eye(3);
    Q_c = (2*simpar.nav.params.sig_c_ss^2/simpar.general.tau_c) * eye(3);
    Q_a = (2*simpar.nav.params.sig_accel_ss^2/simpar.general.tau_b) * eye(3);

    % Assign Q
    Q(1:3,:)   = [Q_n, zeros(3), zeros(3), zeros(3)];
    Q(4:6,:)   = [zeros(3), Q_g, zeros(3), zeros(3)];
    Q(7:9,:)   = [zeros(3), zeros(3), Q_c, zeros(3)];
    Q(10:12,:) = [zeros(3), zeros(3), zeros(3), Q_a];
end
