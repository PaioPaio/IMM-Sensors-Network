function [xLS,PLS] = WLS(z,C,H)
%Weighted least min squared
PLS=inv(H'/C*H);

xLS=PLS*H'/C*z;
end

