function [ Pp ] = update_covariance( P,K,H,R,G,simpar )
%update_covariance updates the covariance matrix
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
% [ output_args ] = update_covariance( input_args )
%
% See also FUNC1, FUNC2

% Author: Randy Christensen
% Date: 31-Aug-2020 16:02:09
% Reference: 
% Copyright 2020 Utah State University

%Don't forget to perform numerical checking and conditioning of covariance
%matrix

IKH = eye(simpar.states.nxfe) - K*H;
Pp  = IKH*P*IKH' + K*G*R*G'*K';

DOS = diag(Pp) <= diag(P);

if ~all(DOS)
    % fprintf('oops!\n');
end

Pp = 0.5*(Pp + Pp');
end
