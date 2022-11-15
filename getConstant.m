function c = getConstant(t_in,t_out,qin,qout,vin,vout,ain,aout)
A = [1, t_in, t_in^2, t_in^3, t_in^4, t_in^5;
     0, 1, 2*t_in, 3*t_in^2, 4*t_in^3, 5*t_in^4;
     0, 0, 2, 6*t_in, 12*t_in^2, 20*t_in^3;
     1, t_out, t_out^2, t_out^3, t_out^4, t_out^5;
     0, 1, 2*t_out, 3*t_out^2, 4*t_out^3, 5*t_out^4;
     0, 0, 2, 6*t_out, 12*t_out^2, 20*t_out^3];

c = zeros(6,4);
for i = 1:4
    b = [qin(i);vin(i);ain(i);qout(i);vout(i);aout(i)];
    c(:,i)=A\b;
end
end