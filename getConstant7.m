function [MC] = getConstant7(tin,tout,qin,qout,vin,vout,ain,aout)
MT = [1 tin tin^2 tin^3 tin^4 tin^5 tin^6 tin^7;
       0 1 2*tin 3*tin^2 4*tin^4 5*tin^5 6*tin^5 7*tin^6;
       0 0 2  6*tin 12*tin^2 20*tin^3 30*tin^4 42*tin^5;
       0 0 0 6 24*tin 60*tin^2 120*tin^3 210*tin^4;
      1 tout tout^2 tout^3 tout^4 tout^5 tout^6 tout^7;
       0 1 2*tout 3*tout^2 4*tout^3 5*tout^4 6*tout^5 7*tout^6;
       0 0 2 6*tout 12*tout^2 20*tout^3 30*tout^4 43*tout^5;
       0 0 0 6 24*tout 60*tout^2 120*tout^3 210*tout^4;];
MQ = [qin vin ain 0 qout vout aout 0];
MC = MT/MQ;
end