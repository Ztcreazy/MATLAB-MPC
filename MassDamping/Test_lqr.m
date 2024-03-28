clear
close all
clc

m = 1; M = 5; L = 2; g = -10; d = 1;
b = -1; % 1 up -1 down
A = [0 1 0 0;
0 -d/M -m*g/M 0;
0 0 0 1;
0 -b*d/(M*L) -b*(m+M)*g/(M*L) 0];
B = [0; 1/M; 0; b*1/(M*L)];
C = [1 0 0 0];
D = 0;
sys = ss(A,B,C,D);
Ts = 0.001;
sys_d = c2d(sys,Ts);

A_d = sys_d.A;
B_d = sys_d.B;
C_d = sys_d.C;
D_d = sys_d.D;

Q = [1 0 0 0;...
     0 1 0 0;...
     0 0 10 0;...
     0 0 0 100];
R = 0.001; % control cost

Kr = lqr(sys,Q,R);
Kr_d = dlqr(A_d,B_d,Q,R);
Kr_c = lqrd(A,B,Q,R,Ts);
% K_f = LQR_function(A_d,B_d,Q,R,Q);

%%
Pf = Q;
N = 15000; % 1500
for i = 1 : N
    K = (B_d'*Pf*B_d + R) \ B_d'*Pf*A_d;
    % V(1) + V(2) = V
    % Pf = Q + A_d'*Pf*A_d - A_d'*Pf*B_d*inv(B_d'*Pf*B_d + R)*B_d'*Pf*A_d;
    % Derivative
    Pf = Q + (A_d - B_d*K)'*Pf*(A_d - B_d*K) + K'*R*K;
    if i == 1
        K_gain = K;
    else
        K_gain = [K; K_gain];
    end
end