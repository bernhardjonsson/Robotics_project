function [x,y,z] = forwardKinematics(q2,q3,q4)
	% calculates the 3D coordinates of the end effector based on the angles

L12 = 93;
L23 = 93;
L34 = 50;

% hliðra q2 um 90 deg

r3 = L34*sin(q3+q4);
r2 = r3 + L23*sin(q3);

r1 = r3 + r2 +L12 % this is the total length of the arm


y = r1 * cos(q2)





