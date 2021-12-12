% Input:
%   q1: First quaternion
%   q2: Second quaternion
%
% Output:
%   q: Multiplication of qaternion 1 and 2
%
function [q] = quatmult(q1,q2)
    % Scalar values
    q1_s = q1(1);
    q2_s = q2(1);

    % Vectors
    q1_v = q1(2:4);
    q2_v = q2(2:4);

    q = [q1_s*q2_s - dot(q1_v, q2_v);
         q1_s*q2_v + q2_s*q1_v - dot(q1_v,q2_v)];
end
