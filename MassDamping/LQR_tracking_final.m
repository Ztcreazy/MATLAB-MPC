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

Ts = 0.1;
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
% delta u_k = u_k - u_k-1
% [x_k+1; x_d_k+1; u_k] = 
% [A 0 B; 0 I 0; 0 0 I]*[x_k; x_d_k; u_k-1] +
% [B; 0; I]*delta u_k
% x_d_k+1 = T*x_d_k;

T = [1 0.1; 0 1];
% uniform speed

A_a = [A_d zeros(n) B_d; zeros(n) T zeros(n,p);...
       zeros(p,n) zeros(p,n) eye(p)];
B_a = [B_d; zeros(n, p); eye(p)];
C_a = [eye(n) -eye(n) zeros(n,p)];

x0 = [0; 0];
x_d = [0; 0.2];
x = x0;
u0 = 0;
u = u0;

% u_d = mldivide(B_d, (eye(n)-A_d)*x_d);
% ud = mldivide(B, (eye(n)-A)*x_d);
x_a = [x; x_d; u];

k_steps = 500;
x_save = zeros(n,k_steps);
x_save(:,1) = x;
x_d_save = zeros(n,k_steps);
x_d_save(:,1) = x_d;

u_save = zeros(p,k_steps);
u_save(:,1) = u;

Q = [1 0; 0 1];
R = 2;
Pf = Q;

Q_a = C_a'*Q*C_a;
Pf_a = C_a'*Pf*C_a;

N = k_steps;
for i = 1 : k_steps
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

    if (i == 100)
        x_d = [x_d(1); -0.2];
    elseif (i == 200)
        x_d = [x_d(1); 0.2];
    elseif (i == 300)
        x_d = [x_d(1); -0.2];
    elseif (i == 400)
        x_d = [x_d(1); 0.2];
    end
    % u = -Kx
    K_f = LQR_function(A_a, B_a, Q_a, R, Pf_a);
    % delta u = u_k - u_d_k
    % u = - K_f * x_a;
    % x_a = A_a * x_a + B_a * u;
    deltau = - K_f * x_a;
    u = deltau + u;
    x = A_d * x + B_d * u;
    x_d = T * x_d;
    x_a = [x; x_d; u];
    x_save(:,i+1) = x;
    x_d_save(:,i+1) = x_d;
    u_save(:,i) = u;
end

subplot(3,1,1)
plot(x_save(1,:),'LineWidth',2)
hold on
plot(x_d_save(1,:),'LineWidth',2)
legend('x1', 'x1d')
xlim([1,k_steps])
grid on

subplot(3,1,2)
plot(x_save(2,:),'LineWidth',2)
hold on
plot(x_d_save(2,:),'LineWidth',2)
legend('x2', 'x2d')
xlim([1,k_steps])
grid on

subplot(3,1,3)
stairs(u_save(1,:),"LineWidth",1.5)
legend('u')
xlim([1,k_steps])
grid on

figure
M = 5;
for k = 1:1:length(x_save)
    drawmass(x_save(1,k),m,M);
end