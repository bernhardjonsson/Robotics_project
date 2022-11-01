% Test that the forward kinematics matrix outputs the position and
% orientation from the beggining

o_04 = [150, 0, 200];

x_04 = [1, 0, 0];


theta1, theta2, theta3, theta4 = invKinematics(x_04, o_04);

T = T04(theta1, theta2, theta3, theta4);

disp('T04 = ')
disp(T)
