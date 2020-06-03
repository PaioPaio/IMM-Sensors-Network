function [G,F] = Fk(x,u,T,s,r)
%linear update matrix for extended kalman filter for unicycle case

Fbs={zeros(length(x));...
    [0 0 0 -u(1)*r*sin(x(4))*T^2/2;
    0 0 0 u(1)*r*sin(x(4))*T^2/2;
    0 0 0 0;
    0 0 0 0];...
    [0 0 0 +u(1)*r*sin(x(4))*T^2/2;
    0 0 0 -u(1)*r*sin(x(4))*T^2/2;
    0 0 0 0;
    0 0 0 0];...
    zeros(length(x));...
    zeros(length(x))};
F=[1 0 T*cos(x(4)) -x(3)*sin(x(4))*T;
    0 1 T*sin(x(4)) x(3)*cos(x(4))*T;
    0 0 1 0
    0 0 0 1]+Fbs{s};
G=[T^2/2*cos(x(4))*r	0;
    T^2/2*sin(x(4))*r	0;
    r*T 0;
    0 T];
end

