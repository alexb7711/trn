function [ T_i2lvlh ] = inertial2lvlh( r_i, v_i )
%inertial2lvlh Summary of the function goes here
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
% [ output_args ] = inertial2lvlh( input_args )
%
% See also FUNC1, FUNC2
%TODO: Add documentation

% Author: Randy Christensen
% Date: 11-May-2020 23:38:13
% Reference: 
% Copyright 2020 Utah State University

ir = r_i/norm(r_i);
h =  cross(r_i,v_i);
ih = h/norm(h);
iv = cross(ih,ir);
T_i2lvlh = [iv ih ir]';
end
