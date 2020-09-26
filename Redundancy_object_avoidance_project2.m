clear all
close all
% Link length
l1 = 2;
l2 = 2;
l3 = 2;


%Initial joint angles in radian

theta1 = 1.7;
theta2 = -0.5;
theta3 = 1;

% Each Joint Positions using forward kinematic. 
    
px1 = l1*cos(theta1);
py1 = l1*sin(theta1);

px2 = l1*cos(theta1)+l2*cos(theta2+theta1);
py2 = l1*sin(theta1)+l2*sin(theta2+theta1);

px3 = l1*cos(theta1)+l2*cos(theta2+theta1)+l3*cos(theta3+theta2+theta1);
py3 = l1*sin(theta1)+l2*sin(theta2+theta1)+l3*sin(theta3+theta2+theta1);
       
% Target Point-calcuated by forward kinematic using the pre-defined joint angles....    

tp = [px3; py3]'; % Target position 

% centre of the circle obstacle

cen_ox = 0.1; % centre of the circle obstacle x position
cen_oy = 2; % centre of the circle obstacle y position

ox = cen_ox;
oy = cen_oy;
  
% Points values surround the circle obstacle to draw circle..
    ang=0:0.01:2*pi; 
    
    oxp=0.4*cos(ang)+cen_ox;
    oyp=0.4*sin(ang)+cen_oy;

% a constant vlaue for increment of q0
    k0 = 0.1;
    
        
% Identity Matrix
    
In = [1 0 0; 0 1 0; 0 0 1];

steps=50;

for i = 1:steps
%Set velocity to zero, only allow internal motion
ve = [0;0;];


% complete the inverse kinematics using Jacobian for the redundant
J=[-sin(theta1)*(l2*cos(theta2)+l3*cos(theta2+theta3)) -cos(theta1)*(l2*sin(theta2)+l3*sin(theta2+theta3)) -l3*cos(theta1)*sin(theta2+theta3);
   cos(theta1)*(l2*cos(theta2)+l3*cos(theta2+theta3))] -sin(theta1)*(l2*sin(theta2)+l3*sin(theta2+theta3)) -l3*sin(theta1)*sin(theta2+theta3);
   0 l2*cos(theta2)+l3*cos(theta2+theta3) l3*cos(theta2+theta3)]
JT=J.';
J1=JT*inv(J*JT);

for i = 0:0.0157:pi;

theta1 = i;
theta2 = i;
theta3 = i;

theta_total(t)=theta1+theta2+theta3;

px(t) = l1*cos(theta1)+l2*cos(theta2+theta1)+l3*cos(theta3+theta2+theta1);
py(t) = l1*sin(theta1)+l2*sin(theta2+theta1)+l3*sin(theta3+theta2+theta1);
 
l1x(t) = l1*cos(theta1)  ;
l1y(t) = l1*sin(theta1);

l2x(t) = l1*cos(theta1)+l2*cos(theta2+theta1);
l2y(t) = l1*sin(theta1)+l2*sin(theta2+theta1);

l3x(t) = l1*cos(theta1)+l2*cos(theta2+theta1)+l3*cos(theta3+theta2+theta1);   
l3y(t) = l1*sin(theta1)+l2*sin(theta2+theta1)+l3*sin(theta3+theta2+theta1);%inverse kinematrcs using Jacobian for the redundant


% manipulator

q=J1*ve



% the distance between P1,P2,P3 and the centre of the obstacle
distance = [sqrt((px1-cen_ox)^2+(py1-cen_oy)^2); sqrt((px2-cen_ox)^2+(py2-cen_oy)^2); sqrt((px3-cen_ox)^2+(py3-cen_oy)^2)];


% select a joint which has minimum distance from the centre of obstacle

[min_dis min_joint]= min(distance);

% use the provided differentiation.m to compute the fomulis dw/dq, and let w_pq=dw/dq

p1 = sqrt((l1*cos(theta1)-ox)^2+(l1*sin(theta1)-oy)^2); % joint 1
dif_p1theta1=diff(p1,theta1)  % differentiate p1 respect to theta1

%joint2
p2 = sqrt((l1*cos(theta1)+l2*cos(theta2+theta1)-ox)^2+( l1*sin(theta1)+l2*sin(theta2+theta1)-oy)^2);
dif_p2theta1=diff(p2,theta1)  % differentiate p2 respect to theta1
dif_p2theta2=diff(p2,theta2)  % differentiate p2 respect to theta2

%joint3
p3 = sqrt((l1*cos(theta1)+l2*cos(theta2+theta1)+l3*cos(theta3+theta2+theta1)-ox)^2+(l1*sin(theta1)+l2*sin(theta2+theta1)+l3*sin(theta3+theta2+theta1)-oy)^2);
dif_p3theta1=diff(p3,theta1)
dif_p3theta2=diff(p3,theta2)
dif_p3theta3=diff(p3,theta3)





%compute the q0dot 
q0dot =; 

%compute qdot





% distances of P1,2,3 to the object
dis_j1(i) = distance(1);
dis_j2(i) = distance(2);
dis_j3(i) = distance(3);

% position error of the endeffector
error_p(i) = sum([abs(tp(1)-px3) abs(tp(2)-py3)]);
   

figure(1),
plot(tp(1), tp(2),'r*')
plot(px3, py3,'k*')
plot(oxp,oyp,'-k','LineWidth',4)
line([0 px1],[0 py1],'Color','r','LineWidth',4)
line([px1 px2],[py1 py2],'Color','g','LineWidth',4)
line([px2 px3],[py2 py3],'Color','b','LineWidth',4)
axis([-3 3,0,6])
drawnow;
pause(0.1)


end



%plotting, you can skip this part
plot_size = steps;
    
    figure(2), 
    hold on
    plot(1:1:plot_size,dis_j1,'r','LineWidth',4)
    plot(1:1:plot_size,dis_j2,'b','LineWidth',4)
    plot(1:1:plot_size,dis_j3,'k','LineWidth',4)
    set(gca,'FontSize',10,'fontWeight','bold')
    grid on
    xlabel('steps','FontSize', 15)
    ylabel('The distace from obstacle','FontSize', 15)

    figure(3), plot(1:1:plot_size,error_p,'r','LineWidth',4)
    hold on
    set(gca,'FontSize',10,'fontWeight','bold')
    grid on
    xlabel('steps','FontSize', 15)
    ylabel('The position error','FontSize', 15)
    
    
 