clear;close all;clc
% Define the convex functions f(x) and g(x)
% For demonstration purposes, let's assume:
% f(x) = ||x||_1 (L1-norm) and g(x) = ||x||_2^2 (squared L2-norm)
% Proximal operator for f(x) (L1-norm)
prox_f = @(x, gamma) sign(x) .* max(abs(x) - gamma, 0);

% Proximal operator for g(x) (squared L2-norm)
prox_g = @(x, gamma) x ./ (1 + 2*gamma);

% Define the parameters
gamma = 0.1; % Step size

% Initial guess
x0 = randn(100, 1);

% Number of iterations
max_iter = 1000;

% Douglas-Rachford iteration
x = x0;
for iter = 1:max_iter
    u = prox_f(x, gamma);
    v = prox_g(2*u - x, gamma);
    x_new = x + v - u;
    
    % Check for convergence
    if norm(x_new - x) < 1e-6
        break;
    end
    
    x = x_new;
end

disp(['Converged in ', num2str(iter), ' iterations.']);