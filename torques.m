function [t2,t3,t4] = torques(q2,q3,q4,F)
	% calculates the torques at each motor for a static position with the given angles and a FN gravitational force upon the tip of the end effector(frame 4)
	% this function will then spit out Nmm

L12 = 93;
L23 = 93;
L34 = 50;

r3 = L34*sin(q2+q3+q4);
r2 = r3 + L23*sin(q2+q3);
r1 = r3 + r2 +L12*sin(q2);

t4 = F*r3;
t3 = F*r2;
t2 = F*r1;






