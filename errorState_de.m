function [ delx_dot ] = errorState_de( delx, input )
%NAVCOV_DE specifies the differential equation that governs the propagation
%of the covariance of the navigation state estimates

%Compute state dynamics matrix
Fhat = calc_F( input.xhat, input);
% G    = calc_G( input.xhat, input);

%Compute Phat_dot
delx_dot = Fhat * delx; % + G(t) + w(t)
end
