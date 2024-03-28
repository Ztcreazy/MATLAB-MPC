clear
close all
clc
% x_k+1 = A*x_k + B*u_k
A = [1 .1; 0 2];
n  = size(A,1);
B = [0; 0.5];
p = size(B,2);
Q = [1 0; 0 1];
R = .1;
F = [1 0; 0 1];

k_steps = 100;
X_K = zeros(n,k_steps);
U_K = zeros(p,k_steps);
X_K(:,1) = [20; -20];

N = 5;
[E,H] = MPC_Matrix(A,B,Q,R,F,N);

for i = 1:k_steps
    U_K(:,i) = Prediction(X_K(:,i),E,H,N,p,X_K(:,1));
    % U_K(:,i) = 0;
    X_K(:,i+1) = A*X_K(:,i) + B*U_K(:,i);
end

subplot(2,1,1)
hold on
for i = 1:size(X_K,1)
    plot(X_K(i,:),"LineWidth",2)
end
legend("x1", "x2")
xlim([1,k_steps])
grid on
hold off
subplot(2,1,2)
hold on
for i = 1:size(U_K,1)
    plot(U_K(i,:),"LineWidth",2)
end
legend("u1")
xlim([1,k_steps])
grid on