function [T01, T12, T23, T34, T35] = T_finder(theta0,theta1,theta2,theta3)
% Finds the decimal representation of all the T matrixes based on the current angles of the joints

L01 = 50;
L12 = 93;
L23 = 93;
L34 = 50;
L45 = 45;
L35 = 35;

% {0} -> {1}
T01 = [cosd(theta0), 0, sind(theta0), 0;
       sind(theta0), 0, -cosd(theta0), 0;
       0, 1, 0, L01;
       0, 0, 0, 1];

% {1} -> {2}
T12 = [cosd(theta1), -sind(theta1), 0, L12*cosd(theta1);
       sind(theta1), cosd(theta1), 0, L12*sind(theta1);
       0, 0, 1, 0;
       0, 0, 0, 1];

% {2} -> {3}
T23 = [cosd(theta2), -sind(theta2), 0, L23*cosd(theta2);
       sind(theta2), cosd(theta2), 0, L23*sind(theta2);
       0, 0, 1, 0;
       0, 0, 0, 1];

% {3} -> {4}
T34 = [cosd(theta3), -sind(theta3), 0, L34*cosd(theta3);
       sind(theta3), cosd(theta3), 0, L34*sind(theta3);
       0, 0, 1, 0;
       0, 0, 0, 1];

% {3} -> {5}
T35 = [1, 0, 0, L35;
       0, 1, 0, L45;
       0, 0, 1, 0;
       0, 0, 0, 1];




end
