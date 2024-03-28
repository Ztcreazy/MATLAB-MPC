function [A,B] = Jacobian(x,u,p)
alpha = u(1);
v = u(2);
theta = x(3);
beta = x(4);
M = p(1);
L1 = p(2);
L2 = p(3);

A = zeros(4,4);
A(1,3) = -v*cos(beta)*sin(theta)*((M*tan(alpha)*tan(beta))/L1 + 1);
A(1,4) = (M*v*cos(beta)*tan(alpha)*cos(theta)*(tan(beta)^2 + 1))/L1...
         - v*sin(beta)*cos(theta)*((M*tan(alpha)*tan(beta))/L1 + 1);
A(2,3) = v*cos(beta)*cos(theta)*((M*tan(alpha)*tan(beta))/L1 + 1);
A(2,4) = (M*v*cos(beta)*tan(alpha)*sin(theta)*(tan(beta)^2 + 1))/L1...
         - v*sin(beta)*sin(theta)*((M*tan(alpha)*tan(beta))/L1 + 1);
A(3,4) = v*(cos(beta)/L2 + (M*tan(alpha)*sin(beta))/(L1*L2));
A(4,4) = -v*(cos(beta)/L2 + (M*tan(alpha)*sin(beta))/(L1*L2));

B = zeros(4,2);
B(1,1) = (M*v*cos(beta)*tan(beta)*cos(theta)*(tan(alpha)^2 + 1))/L1;
B(1,2) = cos(beta)*cos(theta)*((M*tan(alpha)*tan(beta))/L1 + 1);
B(2,1) = (M*v*cos(beta)*tan(beta)*sin(theta)*(tan(alpha)^2 + 1))/L1;
B(2,2) = cos(beta)*sin(theta)*((M*tan(alpha)*tan(beta))/L1 + 1);
B(3,1) = -(M*v*cos(beta)*(tan(alpha)^2 + 1))/(L1*L2);
B(3,2) = sin(beta)/L2 - (M*cos(beta)*tan(alpha))/(L1*L2);
B(4,1) = v*((tan(alpha)^2 + 1)/L1 + (M*cos(beta)*(tan(alpha)^2 + 1))/(L1*L2));
B(4,2) = tan(alpha)/L1 - sin(beta)/L2 + (M*cos(beta)*tan(alpha))/(L1*L2);
