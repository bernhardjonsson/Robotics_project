function T = T04(theta1, theta2, theta3, theta4)

% Function that spits the T_04 matrix (from base to stylus). Is based on
% the solution from exercise 1.

e1 = sind(theta2 + theta3 + theta4);

e3 = cosd(theta2 + theta3 + theta4);

e2 = 50*e3 + 93*cosd(theta2 + theta3) + 93*cosd(theta2);


T = [e3*cosd(theta1), -e1*cosd(theta1), sind(theta1), cosd(theta1)*e2;
     e3*sind(theta1), -e1*sind(theta1), -cosd(theta1), -sind(theta1)*e2;
     e1, e3, 0, 50*e1 + 93*sind(theta2 + theta3) + 93*sin(theta2) + 50;
     0, 0, 0, 1];
end