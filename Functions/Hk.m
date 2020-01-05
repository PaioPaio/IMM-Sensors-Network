function H=Hk(xy,rho)

H=[xy/rho;...
    -xy(2)/rho^2,xy(1)/rho^2];
end

