clearvars
close all
clc
%% Define a vector
v_i = [1,2,3]';
%% Define a transformation
n = [0.1,-0.5,1]';
n = n/norm(n);
th = 24*pi/180;
q_i2b = rotv2quat(th,n)
% q_i2b_alt = [cos(th/2);sin(th/2)*n]
%% Transform using the DCM
T_i2b = q2tmat(q_i2b)
% T_i2b_alt = eye(3) - 2*q_i2b(1)*vx(q_i2b(2:4)) + 2*vx(q_i2b(2:4))*vx(q_i2b(2:4))
%% Transform using the quaternion
v_b = T_i2b * v_i
q_b2i = qConjugate(q_i2b);
v_b_alt = qmult(q_i2b,qmult([0;v_i],q_b2i));
v_b_alt = v_b_alt(2:4)
%% Build a transformation matrix from an euler angle sequence
th_z = 45*pi/180;
th_y = 30*pi/180;
th_x = -25*pi/180;
T_i2b = zyx2dcm([th_z, th_y, th_x])'
%% Build a quaternion from a sequency of angle-axis parameters
u_z = [0,0,1]';
u_y = [0,1,0]';
u_x = [1,0,0]';
q_i2z = rotv2quat(th_z,u_z);
q_z2y = rotv2quat(th_y,u_y);
q_y2x = rotv2quat(th_x,u_x);
q_i2b = qmult(q_y2x,qmult(q_z2y,q_i2z));
T_i2b_alt = q2tmat(q_i2b)