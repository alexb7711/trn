% Input:
%   x     : State vector
%   simpar: Simulation parameters
%
% Output:
%   T: Transformation matrix from moon frame to inertial frame
%
function [T] = Tm2i(q_im)
    % Calculate transformation matrix
    % Local Variables
    q0   = q_im(1);
    q1   = q_im(2);
    q2   = q_im(3);
    q3   = q_im(4);

    % Calculate transormation matrix
    T = [2*(q0^2 + q1^2)-1 , 2*(q1*q2 - q0*q3) , 2*(q1*q3 + q0*q2);
         2*(q1*q2 + q0*q3) , 2*(q0^2 + q2^2)-1 , 2*(q2*q3 - q0*q1);
         2*(q1*q3 - q0*q2) , 2*(q2*q3 + q0*q1) , 2*(q0^2 + q3^2)-1]';
end

