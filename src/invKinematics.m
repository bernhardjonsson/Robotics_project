function [theta1, theta2, theta3, theta4] = invKinematics(x_04, o_04)

% TEST THE INVERSE KINEMATICS

% Lengths
a0 = 50;
a1 = 93;
a2 = 93;
a3 = 50;

theta1 = atan2(o_04(2), o_04(1));

o_03 = o_04 - a3*x_04;

r_13 = sqrt(o_03(1)^2 + (o_03(3)-a0)^2);


theta3 = acos((a1^2 + a2^2 - r_13^2)/(2*a1*a2))

phi = atan2(o_03(3) - a0, o_03(3));

a2*sin(theta3)
a1 + a2*cos(theta3)
psi = atan2(a2*sin(theta3), a1 + a2*cos(theta3));

theta2 = phi + psi;

theta234 = atan2(x_04(3), sqrt(1-x_04(3)^2));

theta4 = theta234 - theta3 - theta2;

% theta4 = asin((o_04(3) - o_03(3))/a3);

disp('Angles:')
disp(' theta1: ')
disp(theta1)
disp(' theta2: ')
disp(theta2)
disp(' theta3: ')
disp(theta3)
disp(' theta4: ')
disp(theta4)

end
