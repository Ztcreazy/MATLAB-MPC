function [F] = LQR_function(A,B,Q,R,S)

n = size(A, 1);

P0 = S;
max_iter = 2000;
P = zeros(n, n*max_iter);
P(:, 1:n) = P0;
P_k_min_1 = P0;
tol = 1e-6;
diff = Inf;
F_N_min_k = Inf;
k = 1;

while diff > tol
    F_N_min_k_pre = F_N_min_k;
    F_N_min_k = (R + B'*P_k_min_1*B)\B'*P_k_min_1*A;
    P_k = (A - B*F_N_min_k)'*P_k_min_1*(A - B*F_N_min_k) + (F_N_min_k)'*R*(F_N_min_k) + Q;
    P(:, n*k-n+1:n*k) = P_k;
    P_k_min_1 = P_k;
    diff = abs(max(F_N_min_k - F_N_min_k_pre));
        k = k+1;
        if k > max_iter
            error("Max iteration !")
        end
 end
F = F_N_min_k;
end