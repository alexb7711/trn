% Input:
%   pos    : Vehicle position
%   vel    : Vehiclde velocity
%   simpar : Simulation parameters
%
% Output:
%   T: Transformation matrix from inertial frame to body frame
%
function [T] = Ti2b(pos, vel, simpar)
    %%---------------------------------------------------------------------------
    % Unit coordinate frame for departure
    b       = simpar.general.b;
    u_betax = simpar.general.u_beta_x;
    u_betay = simpar.general.u_beta_y;
    u_betaz = simpar.general.u_beta_z;

    % Calculate departure direction
    u_beta = [u_betax; u_betay; u_betaz];

    %%---------------------------------------------------------------------------
    % Calculate components of transformation matrix
    % If the norm looks nice, use the velocity vectory
    if norm(vel) > 1e-5
        u_xb = vel/norm(vel);
    % Otherwise it looks like we are early on in the launch, better stick to the
    % active angle
    else
        u_xb = rotation_vector(b, u_beta, pos);
    end % if

    u_yb = cross(-u_xb, u_beta)/norm(cross(u_xb, u_beta));
    u_zb = cross(u_xb, u_yb)/norm(cross(u_xb, u_yb));

    %%---------------------------------------------------------------------------
    % Assign transformation matrix
    T = [u_xb'; u_yb'; u_zb'];
end
