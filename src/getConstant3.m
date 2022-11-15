function [MC] = getConstant3(tin,tout,qin,qout,vin,vout)
MT = [1 tin tin^2 tin^3;
       0 1 2*tin 3*tin^2;
      1 tout tout^2 tout^3;
       0 1 2*tout 3*tout^2];
MQ = [qin vin qout vout];
MC = MT/MQ;
end