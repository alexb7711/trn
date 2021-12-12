% ===============================================================================
% Reference:
% https://math.stackexchange.com/questions/2248413/skew-symmetric-matrix-of-vector
% Input:
%   x : 3x1 unit vector
%
% Output
%   A : 3x3 skew symmetric matrix
%
function A = rotation_vector(x)
    A = [0     , -x(3) , x(2);
         x(3)  , 0     , -x(1);
         -x(2) , x(1)  , 0];
end
