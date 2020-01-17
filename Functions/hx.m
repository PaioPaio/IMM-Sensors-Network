function zmodel = hx(x,caso)
%nonlinear function of the sensor
switch caso
    case 1
        zmodel=[sqrt(x(1)^2+x(2)^2);atan2(x(2),x(1))];
    case 2
end
end

