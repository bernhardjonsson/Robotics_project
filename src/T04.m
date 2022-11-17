function T = T04(theta1, theta2, theta3, theta4)

% Function that spits the T_04 matrix (from base to stylus). Is based on
% the solution from exercise 1.

[T01, T12, T23, T34, ~] = T_finder(theta1, theta2, theta3, theta4);

T = T01*T12*T23*T34;
end