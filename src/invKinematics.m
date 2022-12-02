function [theta1, theta2, theta3, theta4] = invKinematics(x_04, o_04)

% TEST THE INVERSE KINEMATICS

% Lengths
a0 = 50;
a1 = 93;
a2 = 93;
a3 = 50;

theta1 = atan2d(o_04(2), o_04(1));
theta234 = atan2d(x_04(3), sqrt(1-x_04(3)^2));

o_03 = o_04 - a3*x_04';
%disp(o_03)

r3 = sqrt(o_03(1)^2 + o_03(2)^2);



r13 = sqrt(r3^2 + (o_03(3) - a0)^2);


c3 = (-(a1^2 + a2^2) + r13^2)/(2*a1*a2);

theta3 = atan2d(-sqrt(1-c3^2), c3);
%disp(theta3)

phi = atan2d(o_03(3) - a0, r3);

psi = atan2d(a2*sind(-theta3), a1 + a2*cosd(-theta3));

theta2 = phi + psi;

% c2 = ((a1 + a2*cosd(theta3))*r3 + (a2*sind(theta3))*z3)/(r3^2 + z3^2);
% s2 = (a1 + a2*cosd(theta3)*z3 + (a2*sind(theta3))*r3)/(r3^2 + z3^2);
%
% theta2 = atan2d(s2,c2);


theta4 = theta234 - theta3 - theta2;

% theta4 = asin((o_04(3) - o_03(3))/a3);

%disp('Angles:')
%disp(' theta1: ')
%disp(theta1)
%disp(' theta2: ')
%disp(theta2)
%disp(' theta3: ')
%disp(theta3)
%disp(' theta4: ')
%disp(theta4)

end
