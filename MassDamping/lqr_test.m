clear
clc
% 4*4
A = [1 0 0 0; 1 1 0 0; 1 1 1 0; 1 1 1 1];
% 4*2
B = [1 1 1 1; 0 1 0 2]';
% 4*4
C = [0 0 0 1; 0 0 1 1; 0 1 1 1];
D = 0;
nx = size(A,1);
nu = size(B,2);
ny = size(C,1);
q = 1;
r = 1;
Q = q*eye(nx);
R = r*eye(nu);
K = lqr(A,B,Q,R);

% Ts !!
Ts = 0.001;
sys = ss(A,B,C,D);
sys_d = c2d(sys, Ts);

A_d = sys_d.A;
B_d = sys_d.B;
K_d = dlqr(A_d,B_d,Q,R);
K_c = lqrd(A,B,Q,R,Ts);
