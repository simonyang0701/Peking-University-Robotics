%%this functon transcc2D provide the homogeneous transformation matrix A
%%and the translation vector o1

function [A,o1]=transcc2D(theta) 
alpha = pi/2-theta/2;
l=10;
d=1;

%complete the translation vector o1 and rotation matrix R
o1=[2*l/theta*cos(alpha)*cos(alpha),2*l/theta*cos(alpha)*sin(alpha)]';
%l=(l1+l2)/2 theta=l/d oo1=2(r+d)cos(alpha)
R=[cos(-theta), -sin(-theta);
   sin(-theta), cos(-theta)];

%complete the homogenous transformation matrix
A=[R o1;0 0 1];

end





