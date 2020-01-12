function [xk1,Pk1,dz,S] = kalman(Mat,x,u,z,Pk,Q,R)
%KALMAN Summary of this function goes here
%   Detailed explanation goes here
    xpri=Mat{1}*x+Mat{2}*u;
    Ppri=Mat{1}*Pk*Mat{1}'+Mat{3}*Q*Mat{3}';
    H=Hk(xpri);
    dz=z-H*xprio;
    S=H*Ppri*H'+R;
    W=Ppri*H'*inv(S);
    xk1=xpri+W*dz;
    Pk1=Ppri-W*S*W';
end

        