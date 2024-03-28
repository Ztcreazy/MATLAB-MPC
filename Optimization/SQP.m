clear;close all;clc
% Define the objective function
fun = @(x) (x(1)-1)^2 + (x(2)-2)^2;

% Initial guess
x0 = [0; 0];

% Solve the optimization problem using SQP algorithm
options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'iter');
[x, fval, exitflag, output] = fmincon(fun, x0, [], [], [], [], [], [], @nonlcon, options);

% Display the results
disp('Optimal solution:');
disp(x);
disp('Optimal objective value:');
disp(fval);
disp('Exit flag:');
disp(exitflag);

% Define the nonlinear constraint function
function [c, ceq] = nonlcon(x)
    % Nonlinear inequality constraint
    c = x(1)^2 + x(2)^2 - 1;
    % Nonlinear equality constraint
    ceq = x(1) + x(2) - 2;
end