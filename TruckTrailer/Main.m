clear;close all;clc
%% state space & Jacobian
% syms x y v beta M L1 L2 alpha theta
% f1 = v*cos(beta)*(1 + M/L1*tan(beta)*tan(alpha))*cos(theta);
% f2 = v*cos(beta)*(1 + M/L1*tan(beta)*tan(alpha))*sin(theta);
% f3 = v*(sin(beta)/L2 - M/(L1*L2)*cos(beta)*tan(alpha));
% f4 = v*(tan(alpha)/L1 - sin(beta)/L2 + M/(L1*L2)*cos(beta)*tan(alpha));
% f = [f1;f2;f3;f4];
% gradient(f1,beta);
% J1 = jacobian(f, [x, y, theta, beta]);
% J2 = jacobian(f, [alpha, v]);
% J1 = [jacobian(f1, [x, y, theta, beta]);...
%       jacobian(f2, [x, y, theta, beta]);...
%       jacobian(f3, [x, y, theta, beta]);...
%       jacobian(f4, [x, y, theta, beta]);];
% J2 = [jacobian(f1, [alpha, v]);...
%       jacobian(f2, [alpha, v]);...
%       jacobian(f3, [alpha, v]);...
%       jacobian(f4, [alpha, v]);];
%%
% M (hitch length)
% L1 (truck length)
% W1 (truck width)
% L2 (trailer length)
% W2 (trailer width)
% Lwheel (wheel diameter)
% Wwheel (wheel width)
parameters = struct('M',1,'L1',6,'W1',2.5,'L2',10,...
                    'W2',2.5,'Lwheel',1,'Wwheel',.4);
initial_p = [-40;-20;0;0];
target_p = [0;-25;pi/2;0];
Truck_trailer_plot(initial_p,target_p,parameters)

u0 = zeros(2,1);

inequality_c = Inequality_constraints(1,initial_p,u0,...
    [parameters.M;parameters.L1;parameters.L2;parameters.W1;parameters.W2]);
if any(inequality_c > 0)
    fprintf('Initial pose is not valid.\n');
    return
end

%%
p = 60;
nlobj = nlmpcMultistage(p,4,2);
nlobj.Ts = 0.5;

nlobj.Model.StateFcn = 'Dynamics';
nlobj.Model.StateJacFcn = 'Jacobian';
nlobj.Model.ParameterLength = 3;

nlobj.MV(1).Min = -pi/4; % Minimum steering angle
nlobj.MV(1).Max =  pi/4; % Maximum steering angle
nlobj.MV(2).Min = -10; % Minimum velocity (reverse)
nlobj.MV(2).Max =  10; % Maximum velocity (forward)

nlobj.States(4).Min = -pi/3;
nlobj.States(4).Max = pi/3;

for ct=1:p
    nlobj.Stages(ct).CostFcn = "Cost";
    nlobj.Stages(ct).CostJacFcn = "Cost_gradient";
    nlobj.Stages(ct).ParameterLength = 5;
end

for ct=2:p
    nlobj.Stages(ct).IneqConFcn = "Inequality_constraints";
end

nlobj.Model.TerminalState = zeros(4,1);

simulation_data = getSimulationData(nlobj,'TerminalState');
simulation_data.StateFcnParameter = [parameters.M;parameters.L1;parameters.L2];
simulation_data.StageParameter = repmat([parameters.M;parameters.L1;parameters.L2;parameters.W1;parameters.W2],p,1);
simulation_data.TerminalState = target_p;

validateFcns(nlobj,[2;3;0.5;0.4],[0.1;0.2],simulation_data)

[simulation_data.InitialGuess, XY0] = Initial_guess(initial_p,target_p,u0,p);

%%
fprintf('Automated Parking Planner is running...\n');
tic;
[~,~,info] = nlmpcmove(nlobj,initial_p,u0,simulation_data);

t=toc;
fprintf('Calculation Time = %s; Objective cost = %s; ExitFlag = %s; Iterations = %s\n',...
        num2str(t),num2str(info.Cost),num2str(info.ExitFlag),num2str(info.Iterations));

Truck_trailer_plot(initial_p, target_p, parameters, info, XY0);