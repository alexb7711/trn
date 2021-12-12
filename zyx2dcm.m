function [ cba ] = zyx2dcm( ypr )
%ypr2dcm computes the transformation matrix from a ZYX euler angle sequence
% The ZYX euler angle sequence begins at the A frame and
%undergoes a yaw rotation about the Z axis, followed by a pitch rotation
%about the once-rotated Y axis, followed by a roll rotation about the
%twice-rotated X axis.
%
% Inputs:
%   ypr = ZYX euler angle sequenc (radians)
%
% Outputs
%   cba = DCM from the B frame to the A frame (unitless)
%
% Example Usage
% [ cba ] = ypr2dcm( ypr )
%

% Author: Randy Christensen
% Date: 06-Feb-2019 11:22:40
% Reference: Strapdown Navigation Second Edition, Paul Savage, section
% 3.2.3.1
% Copyright 2018 Utah State University

%Extract angles
psi = ypr(1);
theta = ypr(2);
phi = ypr(3);

%Compute sines and cosines
cpsi = cos(psi);
spsi = sin(psi);
cth = cos(theta);
sth = sin(theta);
cphi = cos(phi);
sphi = sin(phi);

%Package up output
c11 = cth*cpsi;
c12 = -cphi*spsi + sphi*sth*cpsi;
c13 = sphi*spsi + cphi*sth*cpsi;

c21 = cth*spsi;
c22 = cphi*cpsi + sphi*sth*spsi;
c23 = -sphi*cpsi + cphi*sth*spsi;

c31 = -sth;
c32 = sphi*cth;
c33 = cphi*cth;

%Compute dcm elements
cba = [c11, c12, c13;
       c21, c22, c23;
       c31, c32, c33];
end
