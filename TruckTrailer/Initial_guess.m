function [z0, XY0] = Initial_guess(initial_p,target_p,u0,p)
% Generate initial guess for decision variables used by the path planner of
% a truck and trailer system.

% Copyright 2020 The MathWorks, Inc.

% if y0 > -5 (truck above the obstacle), one way-point is used
if initial_p(2) >= -5
    p1 = round(p/2);
    p2 = p - p1 + 1;
    xMiddle = 0; 
    yMiddle = 10;
    thetaMiddle = pi/4;
    xGuess = [linspace(initial_p(1),xMiddle,p1),linspace(xMiddle,target_p(1),p2)];
    yGuess = [linspace(initial_p(2),yMiddle,p1),linspace(yMiddle,target_p(2),p2)];
    thetaGuess = [linspace(initial_p(3),thetaMiddle,p1),linspace(thetaMiddle,target_p(3),p2)];
% if y0 < -10 (truck below the obstacle), two way-points are used
else
    p1 = round(p/3);
    p2 = round(p/3);
    p3 = p - p1 -p2 + 1;
    x1 = initial_p(1)+sign(initial_p(1))*10; 
    y1 = 10;
    theta1 = pi/6;
    x2 = 0; 
    y2 = 10;
    theta2 = pi/3;
    xGuess = [linspace(initial_p(1),x1,p1),linspace(x1,x2,p2),linspace(x2,target_p(1),p3)];
    yGuess = [linspace(initial_p(2),y1,p1),linspace(y1,y2,p2),linspace(y2,target_p(2),p3)];
    thetaGuess = [linspace(initial_p(3),theta1,p1),linspace(theta1,theta2,p2),linspace(theta2,target_p(3),p3)];
end
betaGuess = zeros(1,p+1);
stateGuess = [xGuess;yGuess;thetaGuess;betaGuess];
z0 = [];
for ct=1:p
    z0 = [z0;stateGuess(:,ct);u0];
end
z0 = [z0;stateGuess(:,ct)];
XY0 = stateGuess(1:2,:)';