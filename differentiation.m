
% theta 1 2 and 3 are joint angles, l1 l2 l3 link length, and ox oy are centre of the obstacle. 

syms theta1  theta3 theta2 l1 l2 l3 ox oy  

p1 = sqrt((l1*cos(theta1)-ox)^2+(l1*sin(theta1)-oy)^2); % joint 1
diff(p1,theta1)  % differentiate p1 respect to theta1

%joint2
p2 = sqrt((l1*cos(theta1)+l2*cos(theta2+theta1)-ox)^2+( l1*sin(theta1)+l2*sin(theta2+theta1)-oy)^2);
diff(p2,theta1)  % differentiate p2 respect to theta1
diff(p2,theta2)  % differentiate p2 respect to theta2

%joint3
p3 = sqrt((l1*cos(theta1)+l2*cos(theta2+theta1)+l3*cos(theta3+theta2+theta1)-ox)^2+(l1*sin(theta1)+l2*sin(theta2+theta1)+l3*sin(theta3+theta2+theta1)-oy)^2);
diff(p3,theta1)
diff(p3,theta2)
diff(p3,theta3)


