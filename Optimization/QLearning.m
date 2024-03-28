clear;close all;clc
% Define parameters
num_states = 5; % Number of states
num_actions = 4; % Number of actions (up, down, left, right)
num_episodes = 1000; % Number of episodes
alpha = 0.1; % Learning rate
gamma = 0.9; % Discount factor
epsilon = 0.1; % Epsilon for epsilon-greedy policy

% Initialize Q-table with arbitrary values
Q = rand(num_states, num_actions);

% Loop through episodes
for episode = 1:num_episodes
    % Initialize state
    state = randi([1, num_states]);
    
    % Loop through steps within episode
    while true
        % Choose action using epsilon-greedy policy
        if rand < epsilon
            action = randi([1, num_actions]);
        else
            [~, action] = max(Q(state, :));
        end
        
        % Simulate action and observe reward and next state (simplified for demonstration)
        reward = simulate_action(state, action);
        next_state = randi([1, num_states]);
        
        % Update Q-value using Q-learning update rule
        Q(state, action) = Q(state, action) + alpha * (reward + gamma * max(Q(next_state, :)) - Q(state, action));
        
        % Break if reached terminal state
        if is_terminal_state(next_state)
            break;
        end
        
        % Update state
        state = next_state;
    end
end

% Display the learned Q-table
disp('Learned Q-table:');
disp(Q);

%%
% Greedy policy
[~, optimal_actions] = max(Q, [], 2);

% Epsilon-greedy policy (optional)
epsilon = 0.1; % Adjust epsilon as needed
optimal_actions_epsilon = zeros(size(optimal_actions));
for state = 1:num_states
    if rand < epsilon
        % Choose a random action
        optimal_actions_epsilon(state) = randi([1, num_actions]);
    else
        % Choose the action with the highest Q-value
        optimal_actions_epsilon(state) = optimal_actions(state);
    end
end

% Display the optimal policy
disp('Optimal Policy (Greedy):');
disp(optimal_actions);

disp('Optimal Policy (Epsilon-Greedy):');
disp(optimal_actions_epsilon);

% Function to simulate action (simplified for demonstration)
function reward = simulate_action(state, action)
    % Define rewards for each state-action pair (simplified)
    rewards = [0, -1, -1, -1; % State 1
               -1, -1, -1, 0; % State 2
               -1, -1, 0, -1; % State 3
               -1, 0, -1, -1; % State 4
               0, -1, -1, -1]; % State 5
    reward = rewards(state, action);
end

% Function to check if state is terminal (simplified for demonstration)
function is_terminal = is_terminal_state(state)
    terminal_states = [1, 5]; % Terminal states
    is_terminal = ismember(state, terminal_states);
end