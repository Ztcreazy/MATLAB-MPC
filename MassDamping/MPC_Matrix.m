function [E,H] = MPC_Matrix(A,B,Q,R,F,N)
n = size(A,1);
p = size(B,2);

M = [eye(n);zeros(n*N,n)];
C = zeros(n*(N+1),N*p);

T = eye(n);

for i = 1:N
    rows = i*n + (1:n);
    C(rows,:) = [T*B,C(rows-n,1:end-p)];
    T = A*T;
    M(rows,:) = T;
end

% kron(N*N,n*n) -> nN*nN
Q_bar = kron(eye(N),Q);
% n*(N+1)
Q_bar = blkdiag(Q_bar,F);
R_bar = kron(eye(N),R);

G = M'*Q_bar*M;
E = M'*Q_bar*C; % C'*Q_bar*M;
H = C'*Q_bar*C + R_bar;
end
