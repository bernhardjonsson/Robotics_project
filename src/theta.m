function [t1,t2,t3] = theta(x,y,z)
L01 = 50;
L12 = 93;
L23 = 93;
D = (x.^2 + y.^2 - L12^2 + (z - L01).^2 - L12^2 - L23^2)/(2*L12*L23)

% we migt need to refine the functions here. Based on math from Taras


t1 = atan2(x,y);
t3 = atan2(D, sqrt(1-D.^2));
t2 = atan2(sqrt(x.^2 + y.^2 - L01^2) , z - L01)- atan2(L12+L23*cos(t3), L23*sin(t3));



end