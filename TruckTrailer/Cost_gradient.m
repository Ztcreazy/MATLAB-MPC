function [Gx,Gmv] = Cost_gradient(stage,x,u,p)

Gx = zeros(4,1);
w = eye(2);
Gmv = 2*w*u;