function [ P_i ] = calcPosVelCov( r_i0, v_i0, sig_at, sig_ct, ...
    sig_alt, sig_atdot, sig_ctdot, sig_altdot, varargin )
%calcPosVelCov Summary of the function goes here
%   Detailed explanation goes here
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
% [ output_args ] = calcPosVelCov( input_args )
%
% See also FUNC1, FUNC2

% Author: Randy Christensen
% Date: 11-May-2020 23:38:32
% Reference: 
% Copyright 2020 Utah State University

%Chaser Position and Velocity
T = inertial2lvlh(r_i0, v_i0);

% Get the component pos/vel variances and correlation
% Range refers to the down-range direction
vr = sig_at^2;
vc = sig_ct^2;
va = sig_alt^2;
vrr = sig_atdot^2;
vrc = sig_ctdot^2;
vra = sig_altdot^2;

if length(varargin) >= 1
    rho  = varargin{1};
else
    rho  = -0.9;
end


Pc_lvlh = [vr            0           0            0            0   rho*sqrt(vr*vra) 
            0           vc           0            0     rho*sqrt(vc*vrc)  0
            0            0           va     rho*sqrt(va*vrr)  0           0
            0            0   rho*sqrt(va*vrr)   vrr           0           0
            0     rho*sqrt(vc*vrc)  0            0           vrc          0
     rho*sqrt(vr*vra)   0           0            0            0          vra];
 
% Transform lvlh covariance to inertial
O33 = zeros(3,3);
TT = [T' O33
      O33 T'];
P_i_full = TT*Pc_lvlh*TT';
P_i.rr = P_i_full(1:3,1:3);
P_i.rv = P_i_full(1:3,4:6);
P_i.vr = P_i_full(4:6,1:3);
P_i.vv = P_i_full(4:6,4:6);

end
