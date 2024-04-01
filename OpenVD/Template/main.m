clear; close all; clc

addpath("C:/Users/14404/OneDrive/Desktop/Optimization/openvd-master/inst")

TireModel = TirePacejka();                  % Choosing tire
VehicleModel = VehicleSimpleNonlinear();    % Choosing vehicle

VehicleModel.tire = TireModel;

T = 6;                              % Total simulation time [s]
resol = 50;                         % Resolution
TSPAN = 0:T/resol:T;                % Time span [s]

simulator = Simulator(VehicleModel, TSPAN);

simulator.ALPHAT0 = -0.2;           % Initial side slip angle [rad]
simulator.dPSI0 = 0.7;              % Initial yaw rate [rad/s]

simulator.Simulate();

XT = simulator.XT;
YT = simulator.YT;
PSI = simulator.PSI;
VEL = simulator.VEL;
ALPHAT = simulator.ALPHAT;
dPSI = simulator.dPSI;

f1 = figure(1);
grid on ; box on
plot(TSPAN,XT,'linewidth',1)
xlabel('time [s]')
ylabel('Distance in the x direction [m]')

f2 = figure(2);
grid on ; box on
plot(TSPAN,YT,'linewidth',1)
xlabel('time [s]')
ylabel('Distance in the y direction [m]')

f3 = figure(3);
grid on ; box on
plot(TSPAN,PSI,'linewidth',1)
xlabel('time [s]')
ylabel('Yaw angle [rad]')

f4 = figure(4);
grid on ; box on
plot(TSPAN,VEL,'linewidth',1)
xlabel('time [s]')
ylabel('Velocity [m/s]')

f5 = figure(5);
grid on ; box on
plot(TSPAN,ALPHAT,'linewidth',1)
xlabel('time [s]')
ylabel('Vehicle slip angle [rad/s]')

f6 = figure(6);
grid on ; box on
plot(TSPAN,dPSI,'linewidth',1)
xlabel('time [s]')
ylabel('Yaw rate [rad/s]')

%%
g = Graphics(simulator);

g.Frame();
g.Animation();
