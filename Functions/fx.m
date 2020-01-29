function x1 = fx(x,u,delta,s,r)
A=[1 0 delta*cos(x(4)) 0;
    0 1 delta*sin(x(4)) 0;
    0 0 1 0
    0 0 0 1];
B={[0 0 0 0;0 0 0 0]',[delta^2/2*cos(x(4))*r delta^2/2*sin(x(4))*r delta*r 0;0 0 0 0]',...
    [-delta^2/2*cos(x(4))*r -delta^2/2*sin(x(4))*r delta*r 0;0 0 0 0]',[0 0 0 0;0 0 0 delta]',[0 0 0 0;0 0 0 -delta]'};
x1=A*x+B{s}*u;
end

