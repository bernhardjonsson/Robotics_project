function t4 = theta4(t1,t2)

R01 = [1 0 0 0;
       0 cos(t1) -sin(t1) 0;
       0 sin(t1) cos(t1) 0;
       0 0 0 1];
% {1} -> {2}
R12 = [cos(t2) -sin(t2) 0 0;
       sin(t2) cos(t2) 0 0;
       0 0 1 0;
       0 0 0 1];

R = R01*R12;


t4 =atan2(cos(t1)*cos(t2+t3)*R(1,3) + sin(t1)*cos(t2+t3)*R(2,3) + sin(t2+t3)*R(3,3), -cos(t1)*sin(t2+t3)*R(1,3) - sin(t1)*sin(t2+t3)*R(2,3)+cos(t2+t3)*R(3,3));

end