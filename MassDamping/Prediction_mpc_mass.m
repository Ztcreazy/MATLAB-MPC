function u_k = Prediction_mpc_mass(x_k,E,H,N,p,x0)

U_k = zeros(p*N,1);
options = optimoptions('quadprog','Display','iter');
A = [];
b = [];
Aeq = [];
beq = [];
lb = []; 
ub = [];
U_k = quadprog(H,E'*x_k,A,b,Aeq,beq,lb,ub,x0,options);
u_k = U_k(1:p,1);
end