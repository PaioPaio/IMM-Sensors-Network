function [xk1,Pk1,dz,S] = kalman(Mat,x,u,z,Pk,Q,R,caso,offset,delta,s)
%KALMAN 
    %prediction
    switch caso
        case 1
            %linear model of movement
            xpri=Mat{1}*x+Mat{2}*u;
            Ppri=Mat{1}*Pk*Mat{1}'+Mat{3}*Q*Mat{3}';
        case 2
            %non linear
            [G,F]=Fk(x,u,delta,s,Mat);
            xpri=fx(x,u,delta,s,Mat);
            Ppri=F*Pk*F'+G*Q*G';
    end
    %jacobian
    H=Hk(xpri,caso,offset);
    %residual
    dz=z-hx(xpri,caso,offset);
    dz=aggiusta(dz);
    S=H*Ppri*H'+R;  %covariance update
    if rcond(S)<0.001
        "ayayaya"
    elseif det(S)>100
        "oyoyoyo"
    end
    W=(Ppri*H')/S;    %kalman weight
    %update
    xk1=xpri+W*dz;
    Pk1=(eye(length(x))-W*H)*Ppri;
end