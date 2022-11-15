function T = T04(theta1, theta2, theta3, theta4)

% Function that spits the T_04 matrix (from base to stylus). Is based on
% the solution from exercise 1.

e1 = sind(theta2 + theta3 + theta4);

e2 = cosd(theta2 + theta3 + theta4);

e4 = 93*sind(theta2)*sind(theta3);

e5 = 93*cosd(theta2)*cosd(theta3);

e3 = e4 - e5 - 93*sind(theta2) + 50*cosd(theta2)*sind(theta3)*sind(theta4) + 50*cosd(theta3)*sind(theta2)*sind(theta4) + 50*sind(theta2)*sind(theta3)*sind(theta4) - 50*cosd(theta2)*cosd(theta3)*sind(theta4);

T = [e2*cosd(theta1), -e1*cosd(theta1), sind(theta1), -cosd(theta1)*e3;
     e2*sind(theta1), -e1*sind(theta1), -cosd(theta1), -sind(theta1)*e3;
     e1, e2, 0, 93*sind(theta1) + e5 + e4 + 50*sind(theta4)*(cosd(theta2)*sind(theta3) + cosd(theta3)*sind(theta2)) + 50*sind(theta4)*(cosd(theta2)*cosd(theta3) - sind(theta2)*sind(theta3)) + 50;
     0, 0, 0, 1];
end