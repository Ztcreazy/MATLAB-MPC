function inequality_c = Inequality_constraints(stage,x,u,p)
% Inequality constraint function of the path planner of a truck and trailer
% system, used to avoid two static obstacles.

% Copyright 2020 The MathWorks, Inc.

%#codegen
%% Ego and obstacles
persistent obstacles truck trailer
M = p(1);
L1 = p(2);
L2 = p(3);
W1 = p(4);
W2 = p(5);
safetyDistance = 1;
if isempty(obstacles)
    %% Truck and trailer
    truck = collisionBox(L1,W1,0);
    trailer = collisionBox(L2,W2,0);
    %% Obstacles
    obs1 = collisionBox(17.5,20,0);
    T1 = trvec2tform([-11.25,-20, 0]);
    obs1.Pose = T1;
    obs2 = collisionBox(17.5,20,0);
    T2 = trvec2tform([11.25,-20, 0]);
    obs2.Pose = T2;
    obstacles = {obs1,obs2};
end
%% constraints
% Update trailer positions
xTrailer = x(1) + L2/2*cos(x(3));
yTrailer = x(2) + L2/2*sin(x(3));
T = trvec2tform([xTrailer,yTrailer,0]);
H = axang2tform([0 0 1 x(3)]);
trailer.Pose = T*H;
% Update truck positions
theta1 = x(3) + x(4);
xTruck = x(1) + L2*cos(x(3)) + (M+L1/2)*cos(theta1);
yTruck = x(2) + L2*sin(x(3)) + (M+L1/2)*sin(theta1);
T = trvec2tform([xTruck,yTruck,0]);
H = axang2tform([0 0 1 theta1]);
truck.Pose = T*H;
% Calculate distances from trailer to obstacles
numObstacles = numel(obstacles);
distances1 = zeros(numObstacles,1);
distances2 = zeros(numObstacles,1);
for ct = 1:numObstacles
    distances1(ct) = localCheckCollision(trailer,obstacles{ct});
    distances2(ct) = localCheckCollision(truck,obstacles{ct});
end
allDistances = [distances1;distances2];
inequality_c = -allDistances + safetyDistance;

function separationDist = localCheckCollision(geom1, geom2)
%#codegen
needMoreInfo = 1;
if(~coder.target('MATLAB'))
    [collisionStatus, separationDist] = ...
        robotics.core.internal.coder.CollisionGeometryBuildable.checkCollision(...
            geom1.GeometryInternal, geom1.Position, geom1.Quaternion,...
            geom2.GeometryInternal, geom2.Position, geom2.Quaternion,...
            needMoreInfo);
else 
    [collisionStatus, separationDist] = ...
        robotics.core.internal.intersect(...
            geom1.GeometryInternal, geom1.Position, geom1.Quaternion,...
            geom2.GeometryInternal, geom2.Position, geom2.Quaternion,...
            needMoreInfo);
end
if collisionStatus
    separationDist = -10;
end


