function T = T04(theta1, theta2, theta3, theta4)

% Function that spits the T_04 matrix (from base to stylus). Is based on
% the solution from exercise 1.

e1 = sind(theta4)*(cosd(theta3)*(cosd(theta1)*sind(theta2) + cosd(theta2)*sind(theta1)) + sind(theta3)*(cosd(theta1)*cosd(theta2) - sind(theta1)*sind(theta2)));

e3 = cosd(theta3)*(cosd(theta1)*cosd(theta2) - sind(theta1)*sind(theta2)) - sind(theta3)*(cosd(theta1)*sind(theta2) + cosd(theta2)*sind(theta1));

e2 = -cosd(theta4)*(cosd(theta3)*(cosd(theta1)*sind(theta2) + cosd(theta2)*sind(theta1)) + sind(theta3)*(cosd(theta1)*cosd(theta2) - sind(theta1)*sind(theta2))) - sind(theta4)*e3;

T = [e2, e1 - cosd(theta4)*e3, 0, 93*cosd(theta1) - 93*cosd(theta1)*sind(theta2) - 93*cosd(theta2)*sind(theta1);
     0, 0, -1, -50;
     cosd(theta4)*e3 - e1, e2, 0, 93*sind(theta1) + 93*cosd(theta1)*cosd(theta2) - 93*sind(theta1)*sind(theta2) + 50;
     0, 0, 0, 1];
end