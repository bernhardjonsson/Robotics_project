function c = getConstantCubic(t_in,t_out,qin,qout,vin,vout,ain,aout)
A = [1, t_in, t_in^2, t_in^3;
     0, 1, 2*t_in, 3*t_in^2;
     1, t_out, t_out^2, t_out^3;
     0, 1, 2*t_out, 3*t_out^2];

c = zeros(4,4);
for i = 1:4
    b = [qin(i);vin(i);qout(i);vout(i)];
    c(:,i)=A\b;
end
end