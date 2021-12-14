%initialize_covariance computes the initial covariance matrix
%
% Inputs:
%   Input1 = description (units)
%   Input2 = description (units)
%
% Outputs
%   Output1 = description (units)
%   Output2 = description (units)
%
function [ P ] = initialize_covariance( simpar )
s = simpar.nav.ic;

P_pos  = diag([s.sig_rsx  , s.sig_rsy  , s.sig_rsz].^2);
P_vel  = diag([s.sig_vsx  , s.sig_vsy  , s.sig_rsz].^2);
P_cam  = diag([s.sig_thcx , s.sig_thcy , s.sig_thcz].^2);
P_bias = diag([s.sig_ax   , s.sig_ay   , s.sig_az].^2);
P_f1   = diag([s.sig_f1x  , s.sig_f1y  , s.sig_f1z].^2);
P_f2   = diag([s.sig_f2x  , s.sig_f2y  , s.sig_f2z].^2);
P_f3   = diag([s.sig_f3x  , s.sig_f3y  , s.sig_f3z].^2);

P = zeros(simpar.states.nxfe, simpar.states.nxfe);

P(1:3, 1:3)     = P_pos;
P(4:6, 4:6)     = P_vel;
P(7:9, 7:9)     = P_cam;
P(10:12, 10:12) = P_bias;
P(13:15, 13:15) = P_f1;
P(16:18, 16:18) = P_f2;
P(19:21, 19:21) = P_f3;
end
