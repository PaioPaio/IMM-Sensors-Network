function H=Hk(xprior)
%xprior-> row vector containing the prediction of the state (6 elements
%[x,y,vx,vy,ax,ay]
%h(xprior) should be the cartesian transform aka
%rho=sqrt(x^2+y^2)
%theta=atan2(y/x)
%H is the linearized z=Hx
%
rho=sqrt(xprior(1,3)*xprior(1,3)');
H=[xprior(1)/rho,0,xprior(1)/rho,0,0,0;...
    -xprior(3)/rho^2,0,xprior(1)/rho^2,0,0,0];
end

