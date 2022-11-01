function [torque2,torque3,torque4] = torques(t2,t3,t4)
	% calculates the torques at each motor for a static position with the given angles and a 1N gravitational force upon the tip of the end effector(frame 4)

lengthFactor = sin(sqrt(t2^2+t3^2+t4^2));
Torque = 1;

torque4 = Torque * lengthFactor * (L34);
torque3 = Torque * lengthFactor * (L23 + L34);
torque2 = Torque * lengthFactor * (L12 + L23 + L34);


