function [xk1,Pk1,dz,S] = kalman(Mat,x,u,z,Pk,Q,R,caso,offset)
%KALMAN 
    %prediction
    xpri=Mat{1}*x+Mat{2}*u;
    Ppri=Mat{1}*Pk*Mat{1}'+Mat{3}*Q*Mat{3}';
    %jacobian
    H=Hk(xpri,caso,offset);
    %residual
    dz=z-hx(xpri,caso,offset);
    dz=aggiusta(dz);
    S=H*Ppri*H'+R;  %covariance update
    W=Ppri*H'/S;    %kalman weight
    %update
    xk1=xpri+W*dz;
    Pk1=Ppri-W*S*W';
end