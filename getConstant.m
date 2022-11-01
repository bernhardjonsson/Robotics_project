function [MC] = getConstant(tin,tout,qin,qout,vin,vout,ain,aout)
MT = [1 tin tin^2 tin^3 tin^4 tin^5;
       0 1 2*tin 3*tin^2 4*tin^4 5*tin^5;
       0 0 2  6*tin 12*tin^2 20*tin^3;
      1 tout tout^2 tout^3 tout^4 tout^5;
       0 1 2*tout 3*tout^2 4*tout^3 5*tout^4;
       0 0 2 6*tout 12*tout^2 20*tout^3];
MQ = [qin vin ain qout vout aout];
MC = MT/MQ;
end