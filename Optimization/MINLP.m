clear;close all;clc
% Define the objective function
f = [1; 1];

% Linear inequality constraints: -x + y <= 1, x + y <= 2
A = [-1 1; 1 1];
b = [1; 2];

% Integer variables: x, y
intcon = [1, 2];

% Lower and upper bounds for variables: -10 <= x, y <= 10
lb = [-10; -10];
ub = [10; 10];

% Solve the MINLP problem
options = optimoptions('intlinprog', 'Display', 'iter');
[x, fval, exitflag] = intlinprog(f, intcon, A, b, [], [], lb, ub, options);

% Display the results
disp('Optimal solution:');
disp(x);
disp('Optimal objective value:');
disp(fval);
disp('Exit flag:');
disp(exitflag);