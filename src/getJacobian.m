function [J] = getJacobian(point)
%JA:(will be written as function:getJacobian)
load("t1.mat");
load("t2.mat");
load("t3.mat");
load("t4.mat");
z0 = [0;0;1];
z1 = [0;-1;0];
z2 = [0;-1;0];
z3 = [0;-1;0];
o04=[cos(t1(point))*(93*sin(t2(point))+93*sin(t2(point)+t3(point))+50*sin(t2(point)+t3(point)+t4(point)));
    sin(t1(point))*(93*sin(t2(point))+93*sin(t2(point)+t3(point))+50*sin(t2(point)+t3(point)+t4(point)));
    50+93*cos(t2(point))+93*cos(t2(point)+t3(point))+50*cos(t2(point)+t3(point)+t4(point))]
o14=[93*sin(t2(point))+93*sin(t2(point)+t3(point))+50*sin(t2(point)+t3(point)+t4(point));
    0;
    50+93*cos(t2(point))+93*cos(t2(point)+t3(point))+50*cos(t2(point)+t3(point)+t4(point))]
o24=[93*sin(t3(point))+50*sin(t3(point)+t4(point));
    93+93*cos(t3(point))+50*cos(t3(point)+t4(point));
    0]
o34=[93+50*cos(t4(point));
    -50*(t4(point));
    0]
J=[cross(z0,o04) cross(z1,o14) cross(z2,o24) cross(z3,o34);
    z0  z1 z2 z3]
end