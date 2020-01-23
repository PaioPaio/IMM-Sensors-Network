function H=Hk(xprior,caso,offset)
%xprior-> row vector containing the prediction of the state (6 elements
%[x,y,vx,vy,ax,ay]
%h(xprior) should be the cartesian transform aka
%rho=sqrt(x^2+y^2)
%theta=atan2(y/x)
%H is the linearized z=Hx
%
switch caso
    case 1
        vector=xprior(1:2)'-offset;
        rho=sqrt(vector*vector');
        H=[vector(1)/rho,vector(2)/rho,0,0,;...
            -vector(2)/rho^2,vector(1)/rho^2,0,0];
    case 2
         vector=xprior(1:2)'-offset;
        rho=sqrt(vector*vector');
        H=[vector(1)/rho,vector(2)/rho,0,0,0,0;...
            -vector(2)/rho^2,vector(1)/rho^2,0,0,0,0];
end
end