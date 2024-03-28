clear
close all
clc
% mass spring damping
% m*xddot = F - k*x - d*xdot
m = 1;
k = 1;
b = 0.5;

A = [0 1; -k/m -b/m];
B = [0; 1/m];
C = [1 0];
D = 0;
sys = ss(A,B,C,D);

Ts = 0.01;
sys_d = c2d(sys,Ts);
A_d = sys_d.A;
n = size(A_d,1);
B_d = sys_d.B;
p = size(B_d,2);

% J = 1/2(e_N'*S*e_N + sum(...
%     e_N-1'*Q*e_N-1 + u_N-1'*R*u_N-1...
%     + e_N-2'*Q*e_N-2 + u_N-2*R*u_N-2...
%     + e_1'*Q*e_1 + u_1*R*u_1))
%   = 1/2(x_a'*C_a'*S*C_a*x_a + ...)

% x_d_k = A*x_d_k + B*u_d_k
% delta u_k = u_k - u_d_k
% [x_k+1; x_d_k+1] = 
% [A I-A; 0 I]*[x_k; x_d_k] +
% [B; 0]*delta u_k
% x_d = A*x_d + B*u_d

A_a = [A_d eye(n)-A_d; zeros(n) eye(n)];
B_a = [B_d; zeros(n, p)];
C_a = [eye(n) -eye(n)];

x0 = [4; 2];
x_d = [2; 0];
x = x0;
u0 = 0;
u = u0;

u_d = mldivide(B_d, (eye(n)-A_d)*x_d);
ud = mldivide(B, (eye(n)-A)*x_d);
x_a = [x; x_d];

k_steps = 1500;
x_save = zeros(n,k_steps);
x_save(:,1) = x;
u_save = zeros(p,k_steps);
u_save(:,1) = u;

Q = [1 0; 0 1];
R = 2;
Pf = Q;

Q_a = C_a'*Q*C_a;
Pf_a = C_a'*Pf*C_a;

N = k_steps;
for i = 1 : N
    K = (B_a'*Pf_a*B_a + R) \ B_a'*Pf_a*A_a;
    % V(1) + V(2) = V
    % Pf = Q + A_d'*Pf*A_d - A_d'*Pf*B_d*inv(B_d'*Pf*B_d + R)*B_d'*Pf*A_d;
    % Derivative
    Pf_a = Q_a + (A_a - B_a*K)'*Pf_a*(A_a - B_a*K) + K'*R*K;
    if i == 1
        K_gain = K;
    else
        K_gain = [K; K_gain];
    end
end

for i = 1 : k_steps
    % u = -Kx
    K_f = LQR_function(A_a, B_a, Q_a, R, Pf_a);
    % delta u = u_k - u_d_k
    u = - K_f * x_a;
    x_a = A_a * x_a + B_a * u;
    % u = - K_f * x_a + u_d;
    % x = A_d * x + B_d * u;
    % x_a = [x; x_d];
    x_save(:,i+1) = x_a(1:n, 1);
    u_save(:,i) = u + u_d;
end

subplot(2,1,1)
for i = 1:n
    plot(x_save(i,:),"LineWidth",2)
    hold on
end
legend(num2str((1:n)', 'x%d'))
xlim([1,k_steps])
grid on
subplot(2,1,2)
for i = 1:p
    stairs(u_save(i,:),"LineWidth",1)
    hold on
end
legend(num2str((1:p)', 'delta u%d'))
xlim([1,k_steps])
grid on

figure
M = 5;
tspan = 0:.01:25;
% x -> x_a
x_a0 = [4; 2; 2; 0];
% u -> u_k - u_d_k
u = @(y) -K_f*y;
[t,y] = ode45(@(t,y)differential_mass_aug_integration(y,m,k,b,u(y),ud),tspan,x_a0);
for k = 1:5:length(t)
    drawmass(y(k,:),m,M);
end