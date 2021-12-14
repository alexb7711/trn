%compute_H_example calculates the measurement sensitivity matrix
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
% [ output_args ] = compute_H_example( input_args )
%
% See also FUNC1, FUNC2

% Author: Randy Christensen
% Date: 31-Aug-2020 16:04:33
% Reference: 
% Copyright 2020 Utah State University
function [ H ] = compute_H(x, input)
    %----------------------------------------------------------------------------
    % Extract Input Variables
    simpar      = input.simpar;
    s           = extract_state(input.x, simpar, 'nav');
    theta_cross = skew_symmetric(s.cam);
    T_bc        = input.Tbc;
    T_bc_nom    = zyx2dcm([simpar.general.ic.th_c_z, simpar.general.ic.th_c_y, simpar.general.ic.th_c_x])';
    T_ib        = input.Tib;
    T_mi        = input.Tmi;
    rcb         = [simpar.general.rcbx; simpar.general.rcby; simpar.general.rcbz];

    %----------------------------------------------------------------------------
    % Calculate measurements
    l_c1 = T_bc * (T_ib * (T_mi * s.f1 - s.pos) - rcb);
    l_c2 = T_bc * (T_ib * (T_mi * s.f2 - s.pos) - rcb);
    l_c3 = T_bc * (T_ib * (T_mi * s.f3 - s.pos) - rcb);

    %----------------------------------------------------------------------------
    % Calculate dhdl
    dh_dl1 = [1/l_c1(3), 0, -l_c1(1)/l_c1(3)^(2); 0, 1/l_c1(3), -l_c1(2)/l_c1(3)^(2)];
    dh_dl2 = [1/l_c2(3), 0, -l_c2(1)/l_c2(3)^(2); 0, 1/l_c2(3), -l_c2(2)/l_c2(3)^(2)];
    dh_dl3 = [1/l_c3(3), 0, -l_c3(1)/l_c3(3)^(2); 0, 1/l_c3(3), -l_c3(2)/l_c3(3)^(2)];

    %----------------------------------------------------------------------------
    % Skew-symmetric matrices
    lc1_nom       = T_bc_nom*(T_ib*(T_mi*s.f1 - s.pos) - rcb);
    lc2_nom       = T_bc_nom*(T_ib*(T_mi*s.f2 - s.pos) - rcb);
    lc3_nom       = T_bc_nom*(T_ib*(T_mi*s.f3 - s.pos) - rcb);

    lc1_nom_cross = skew_symmetric(lc1_nom);
    lc2_nom_cross = skew_symmetric(lc2_nom);
    lc3_nom_cross = skew_symmetric(lc3_nom);

    %----------------------------------------------------------------------------
    % First feature
    dl_dx1(:,1:3)   = -(eye(3) - theta_cross)*T_bc_nom*T_ib;
    dl_dx1(:,4:6)   = zeros(3);
    dl_dx1(:,7:9)   = lc1_nom_cross;
    dl_dx1(:,10:12) = zeros(3);
    dl_dx1(:,13:15) = (eye(3) - theta_cross)*T_bc_nom*T_ib*T_mi;
    dl_dx1(:,16:18) = zeros(3);
    dl_dx1(:,19:21) = zeros(3);

    %----------------------------------------------------------------------------
    % Second feature
    dl_dx2(:,1:3)   = -(eye(3) - theta_cross)*T_bc_nom*T_ib;
    dl_dx2(:,4:6)   = zeros(3);
    dl_dx2(:,7:9)   = lc2_nom_cross;
    dl_dx2(:,10:12) = zeros(3);
    dl_dx2(:,13:15) = (eye(3) - theta_cross)*T_bc_nom*T_ib*T_mi;
    dl_dx2(:,16:18) = zeros(3);
    dl_dx2(:,19:21) = zeros(3);

    %----------------------------------------------------------------------------
    % Third feature
    dl_dx3(:,1:3)   = -(eye(3) - theta_cross)*T_bc_nom*T_ib;
    dl_dx3(:,4:6)   = zeros(3);
    dl_dx3(:,7:9)   = lc3_nom_cross;
    dl_dx3(:,10:12) = zeros(3);
    dl_dx3(:,13:15) = (eye(3) - theta_cross)*T_bc_nom*T_ib*T_mi;
    dl_dx3(:,16:18) = zeros(3);
    dl_dx3(:,19:21) = zeros(3);

    %----------------------------------------------------------------------------
    % Form dh_dl and dl_dx
    H1 = dh_dl1*dl_dx1;
    H2 = dh_dl2*dl_dx2;
    H3 = dh_dl3*dl_dx3;

    %----------------------------------------------------------------------------
    % Compute H
    H = [H1; H2; H3];
end
