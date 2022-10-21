function [T01, T12, T23, T34, T35] = T_finder(t1,t2,t3,t4)
% Finds the decimal representation of all the T matrixes based on the current angles of the joints


L01 = 50;
L12 = 93;
L23 = 93;
L34 = 50;
L45 = 45;
L35 = 35;


% {0} -> {1}
T01 = [1 0 0 0;
       0 cos(t1) -sin(t1) 0;
       0 sin(t1) cos(t1) L01;
       0 0 0 1];
% {1} -> {2}
T12 = [cos(t2) -sin(t2) 0 L12;
       sin(t2) cos(t2) 0 0;
       0 0 1 0;
       0 0 0 1];
% {2} -> {3}
T23 = [1 0 0 L23;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];
% {3} -> {4}
T34 = [1 0 0 L34;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];
% {3} -> {5}
T35 = [1 0 0 L35;
       0 1 0 L45;
       0 0 1 0;
       0 0 0 1];

       




end