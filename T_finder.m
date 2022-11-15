function [T01, T12, T23, T34, T35] = T_finder(theta0,theta1,theta2,theta3)
% Finds the decimal representation of all the T matrixes based on the current angles of the joints



L01 = 50;
L12 = 93;
L23 = 93;
L34 = 50;
L45 = 45;
L35 = 35;


% Rotation matrices
R_x_90 = [1 0 0;
          0 0 -1;
          0 1 0];
R_z_90 = [0 -1 0;
          1 0 0;
          0 0 1];
R_z_th0 = [cos(theta0) -sin(theta0) 0;
       sin(theta0) cos(theta0) 0;
       0 0 1];
R_z_th1 = [cos(theta1) -sin(theta1) 0;
       sin(theta1) cos(theta1) 0;
       0 0 1];
R_z_th2 = [cos(theta2) -sin(theta2) 0;
       sin(theta2) cos(theta2) 0;
       0 0 1];
R_z_th3 = [cos(theta3) -sin(theta3) 0;
       sin(theta3) cos(theta3) 0;
       0 0 1];

R01 = R_x_90*R_z_th0;
R12 = R_z_90*R_z_th1;
R23 = R_z_th2;
R34 = R_z_th3;
R35 = R_z_th3;

% Translation matrices
t01 = [0 0 L01]';
t12 = [L12 0 0]';
t23 = [L23 0 0]';
t34 = [0 0 L34]';
t35 = [0 0 L35]';

% {0} -> {1}
T01 = [R01 t01;
        zeros(1,3) 1];

% {1} -> {2}
T12 = [R12 t12;
       zeros(1,3) 1];

% {2} -> {3}
T23 = [R23 t23;
       zeros(1,3) 1];

% {3} -> {4}
T34 = [R34 t34;
       zeros(1,3) 1];

% {3} -> {5}
T35 = [R35 t35;
       zeros(1,3) 1];

       




end