clear
close all
clc
% mass spring damping
% m*xddot = F - k*x - d*xdot
m = 1;
k = 1;
b = 0.5;

M = 5;

A = [0 1; -k/m -b/m];
B = [0; 1/m];
C = [1 0];
D = 0;
c = rank(ctrb(A,B));
o = rank(obsv(A,C));
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

k_steps = 1500;
x_save = zeros(n,k_steps);
x_save(:,1) = x;
u_save = zeros(p,k_steps);
u_save(:,1) = u;

Q = [1 0; 0 1];
R = 2;
Pf = Q;

N = k_steps;
for i = 1 : N
    K = (B_d'*Pf*B_d + R) \ B_d'*Pf*A_d;
    % V(1) + V(2) = V
    Pf = Q + A_d'*Pf*A_d - A_d'*Pf*B_d*inv(B_d'*Pf*B_d + R)*B_d'*Pf*A_d;
    % Derivative
    % Pf = Q + (A_d - B_d*K)'*Pf*(A_d - B_d*K) + K'*R*K;
    if i == 1
        K_gain = K;
    else
        K_gain = [K; K_gain];
    end
end

K_d = dlqr(A_d,B_d,Q,R);
K_f = LQR_function(A_d,B_d,Q,R,Pf);

for i = 1 : k_steps
    % u = -Kx
    u = -K_gain((i-1)*p+1:i*p, :) * x;
    % u = 0;
    x = A_d * x + B_d * u;
    x_save(:,i+1) = x;
    u_save(:,i) = u;
end

subplot(2,1,1)
for i = 1:n
    plot(x_save(i,:),'LineWidth',2)
    hold on
end
legend(num2str((1:n)', 'x %d'))
xlim([1,k_steps])
grid on
subplot(2,1,2)
for i = 1:p
    stairs(u_save(i,:),'LineWidth',2)
    hold on
end
legend(num2str((1:p)', 'u %d'))
xlim([1,k_steps])
grid on

figure
tspan = 0:.01:15;

% u -> u
u = @(y) -K_d*y;
% u = @(y) 0;
[t,y] = ode45(@(t,y)differential_mass(y,m,k,b,u(y)),tspan,x0);
for k = 1:1:length(t)
    drawmass(y(k,:),m,M);
end