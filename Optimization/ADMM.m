clear;close all;clc
% Define the objective function
objective = @(x) x(1)^2 + x(2)^2;

% Define the constraint functions
constraint1 = @(x) x(1) + x(2) - 1;
constraint2 = @(x) x(1)^2 + x(2)^2 - 1;

% Initial guess
x = [0.5; 0.5];

% Parameters
rho = 1; % Penalty parameter
max_iter = 1000; % Maximum number of iterations
tol = 1e-6; % Tolerance for convergence

% ADMM iterations
for iter = 1:max_iter
    % Minimize the augmented Lagrangian function
    x_new = fminunc(@(x) objective(x) + (rho/2) * (constraint1(x)^2 + constraint2(x)^2), x);
    
    % Update dual variables (not used in this example)
    
    % Check convergence
    if norm(x_new - x) < tol
        break;
    end
    
    x = x_new;
end

% Print the result
fprintf('Optimal solution: [%.6f, %.6f]\n', x(1), x(2));
fprintf('Optimal objective value: %.6f\n', objective(x));
