function vcross = vx(v)
%vx returns the vector cross product matrix associated with the vector B
%such that C = vx(A)*B = A x B
%
% Inputs:
%   v = 3xn input vectors
%
% Outputs
%   vcross = 3x3xn output cross matrix
%
% Example Usage
% vcross = vx(v)

% Author: Randy Christensen
% Date: 13-May-2020 08:52:26
% Reference: Savage Strapdown Analytics, Eqn 3.1.1-14
% Copyright 2020 Utah State University
%% Validate inputs
[m,n] = size(v);
assert(m == 3 && n == 1,'Incorrect input size')
%% Compute cross vector
vcross = zeros(3,3);
vcross(1,2) = -v(3);
vcross(1,3) =  v(2);
vcross(2,1) =  v(3);
vcross(2,3) = -v(1);
vcross(3,1) = -v(2);
vcross(3,2) =  v(1);
end