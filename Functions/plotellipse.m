function [outputArg1,outputArg2] = plotellipse(cov,stato,onoff)
%PLOTELLIPSE Plot an uncertainty ellipses using the covariace matrix
%   Detailed explanation goes here
[eigenvec, eigenval ] = eig(cov);

% Get the index of the largest eigenvector
[largest_eigenvec_ind_c, r] = find(eigenval == max(max(eigenval)));
largest_eigenvec = eigenvec(:, largest_eigenvec_ind_c);

% Get the largest eigenvalue
largest_eigenval = max(max(eigenval));

% Get the smallest eigenvector and eigenvalue
if(largest_eigenvec_ind_c == 1)
    smallest_eigenval = max(eigenval(:,2));
    smallest_eigenvec = eigenvec(:,2);
else
    smallest_eigenval = max(eigenval(:,1));
    smallest_eigenvec = eigenvec(1,:);
end

% Calculate the angle between the x-axis and the largest eigenvector
angle = atan2(largest_eigenvec(2), largest_eigenvec(1));

% This angle is between -pi and pi.
% Let's shift it such that the angle is between 0 and 2pi
if(angle < 0)
    angle = angle + 2*pi;
end

% Get the coordinates of the data mean


% Get the 95% confidence interval error ellipse
%chisquare_val = 2.4477;
theta_grid = linspace(0,2*pi);
phi = angle;
X0=stato(1);
Y0=stato(2);
a=sqrt(3*largest_eigenval); %chisquare_val*
b=sqrt(3*smallest_eigenval); %chisquare_val*

% the ellipse in x and y coordinates 
ellipse_x_r  = a*cos( theta_grid );
ellipse_y_r  = b*sin( theta_grid );

%Define a rotation matrix
R = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];

%let's rotate the ellipse to some angle phi
r_ellipse = [ellipse_x_r;ellipse_y_r]' * R;

% Draw the error ellipse
if onoff ==1
%plot(r_ellipse(:,1) + X0,r_ellipse(:,2) + Y0,'-')
end
outputArg1 = r_ellipse(:,1) + X0;
outputArg2 = r_ellipse(:,2) + Y0;
end

