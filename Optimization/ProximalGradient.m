clear; close all; clc
% Objective function: f(x) = (1/2) * ||Ax - b||^2
A = randn(10, 5); % Random matrix
b = randn(10, 1); % Random vector

% Regularizer: g(x) = lambda * ||x||_1 (L1-norm regularization)
lambda = 0.1; % Regularization parameter

% Gradient of the objective function
grad_f = @(x) A' * (A * x - b);

% Proximal operator of the regularizer (soft thresholding)
prox_g = @(x, gamma) sign(x) .* max(abs(x) - gamma * lambda, 0);

% Step size
eta = 0.01; % Gradient descent step size
gamma = 1 / (2 * eta); % Proximal operator step size

% Initialization
x = zeros(5, 1); % Initial guess
max_iter = 1000; % Maximum number of iterations

% Proximal gradient descent iteration
for iter = 1:max_iter
    % Gradient descent step
    x_new = x - eta * grad_f(x);
    
    % Proximal operator step
    x = prox_g(x_new, gamma);
    
    % Check for convergence
    if norm(x - x_new) < 1e-6
        break;
    end
end

disp(['Converged in ', num2str(iter), ' iterations.']);
disp('Optimal solution:');
disp(x);