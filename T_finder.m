function [T01, T12, T23, T34, T35] = T_finder(t1,t2,t3,t4)
% Finds the decimal representation of all the T matrixes based on the current angles of the joints


L01 = 50;
L12 = 93;
L23 = 93;
L34 = 50;
L45 = 45;
L35 = 35;

T01 = [cos(t1), 0, sin(t1), 0;
       sin(t1), 0, -cos(t1), 0;
       0, 1, 0, L01;
       0, 0, 0, 1];

T12 = [cos(t2), -sin(t2), 0, L12*sin(t2);
       sin(t2), cos(t2), 0, L12*sin(t2);
       0, 0, 1, 0;
       0, 0, 0, 1];

T23 = [cos(t3), -sin(t3), 0, L23*sin(t3);
       sin(t3), cos(t3), 0, L23*sin(t3);
       0, 0, 1, 0;
       0, 0, 0, 1];

       
T34 = [cos(t4), -sin(t4), 0, L34*sin(t4);
       sin(t4), cos(t4), 0, L34*sin(t4);
       0, 0, 1, 0;
       0, 0, 0, 1];

T35 = [1, 0, 0, L35;
       0, 1, 0, L45;
       0, 0, 1, 0;
       0, 0, 0, 1];

end