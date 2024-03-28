clear
close all
clc
% mass damping MPC
% m*xddot = F - k*x - b*xdot
m = 1;
k = 1;
b = 0.5;

A = [0 1; -k/m -b/m];
B = [0; 1/m];
C = [1 0];
D = 0;
c = ctrb(A,B);
rank(c)
o = obsv(A,C);
rank(o)
sys = ss(A,B,C,D);

Ts = 0.01;
sys_d = c2d(sys,Ts);
A_d = sys_d.A;
n = size(A_d,1);
B_d = sys_d.B;
p = size(B_d,2);

x0 = [4; 2];
x = x0;
u0 = 0;
u = u0;

k_steps = 2500;
X_K = zeros(n,k_steps);
U_K = zeros(p,k_steps);
X_K(:,1) = [4; 2];

Q = [1 0; 0 1];
R = 2;
Pf = Q;

N = 10;
[E,H] = MPC_Matrix(A_d,B_d,Q,R,Pf,N);

for i = 1:k_steps
    U_K(:,i) = Prediction_mpc_mass(X_K(:,i),E,H,N,p,x0);
    X_K(:,i+1) = A_d*X_K(:,i) + B_d*U_K(:,i);
end

subplot(2,1,1)
for i = 1:n
    plot(X_K(i,:),'LineWidth',2)
    hold on
end
legend(num2str((1:n)', 'x %d'))
xlim([1,k_steps])
grid on
subplot(2,1,2)
for i = 1:p
    stairs(U_K(i,:),'LineWidth',2)
    hold on
end
legend(num2str((1:p)', 'u %d'))
xlim([1,k_steps])
grid on

figure
M = 5;
for k = 1:1:length(X_K)
    drawmass_mpc(X_K(1,k),m,M);
end