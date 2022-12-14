clc
clear all

% Test that the forward kinematics matrix outputs the position and
% orientation from the beggining

o_04 = [0, 0, 286]';

x_04 = [0, 0, 1];


[theta1, theta2, theta3, theta4] = invKinematics(x_04, o_04);

T = T04(theta1, theta2, theta3, theta4);

disp('T04 = ')
disp(T)
