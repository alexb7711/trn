% Input:
%   x     : State vector
%   simpar: Simulation parameters
%
% Output:
%   T: Transformation matrix from body frame to camera frame
%
function [T] = Tb2c(x, simpar)
    % Convert array to structure
    s = extract_state(x, simpar, 'nav');

    % Calculate transformation matrix
    theta_cross = skew_symmetric(s.cam);
    T_nom       = zyx2dcm([simpar.general.ic.th_c_z, simpar.general.ic.th_c_y, simpar.general.ic.th_c_x])';
    T           = (eye(3) - theta_cross)*T_nom;
end
