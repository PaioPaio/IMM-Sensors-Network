function [xLS,PLS] = WLS(z,C,H)
%Weighted least min squared
PLS=inv(H'/C*H);
if rcond(PLS)<=1e-8
    "WLS con P troppo piccola"
end
xLS=PLS*H'/C*z;
end

