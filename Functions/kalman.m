function [xk1,Pk1,dz,S] = kalman(Mat,x,u,z,Pk,Q,R)
%KALMAN Summary of this function goes here
%   Detailed explanation goes here
    xpri=Mat{1}*x+Mat{2}*u;
    Ppri=Mat{1}*Pk*Mat{1}'+Mat{3}*Q*Mat{3}';
%     H=[1,0,0,0,0,0;0,1,0,0,0,0];
%     znew=[z(1)*cos(z(2));z(1)*sin(z(2))];
%     dz=znew-H*xpri;
    H=Hk(xpri);
    dz=z-[sqrt(xpri(1)^2+xpri(2)^2);atan2(xpri(2),xpri(1))];
    S=H*Ppri*H'+R;
    W=Ppri*H'/S;
    xk1=xpri+W*dz;
    Pk1=Ppri-W*S*W';
end

        