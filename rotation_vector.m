% ===============================================================================
% Input:
%   phi   : Rotation angle
%   u_phi : Rotation axis of phi in frame A
%   u_A   : Basis vector in frame A
%
% Output
%   u_B : Rotation of phi in fame B
%
function u_B = rotation_vector(phi, u_phi, u_A)
    u_X = skew_symmetric(u_phi);
    u_A = u_A/norm(u_A);
    u_B = (eye(3,3) + sin(phi)*u_X + (1 - cos(phi))*u_X*u_X)*u_A;
end
