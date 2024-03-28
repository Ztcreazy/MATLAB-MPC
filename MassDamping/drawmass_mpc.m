function drawmass_mpc(y,~,M)
x = y(1);

W = 1*sqrt(M/5);  % cart width
H = 0.5*sqrt(M/5); % cart height
wr = .2; % wheel radius
% mr = .3*sqrt(m); % mass radius

y = wr/2 + H/2; % cart vertical position
w1x = x - 0.9*W/2;
w1y = 0;
w2x = x +.9*W/2 - wr;
w2y = 0;

plot([-10 10],[0 0],'k','LineWidth',2)
hold on
% cart
rectangle('Position',[x-W/2,y-H/2,W,H],'Curvature',.1,'FaceColor',[1 0.1 0.1])
% wheel 1
rectangle('Position',[w1x,w1y,wr,wr],'Curvature',1,'FaceColor',[0 0 0])
% wheel 2
rectangle('Position',[w2x,w2y,wr,wr],'Curvature',1,'FaceColor',[0 0 0])
% spring
rectangle('Position',[-4,y-H/5,x-W/2-(-4),0.1],'Curvature',1,'FaceColor',[0 0 0])
% wall
rectangle('Position',[-4,-1,0.1,3],'Curvature',1,'FaceColor',[1 1 1])
% pendulum
% plot([x px],[y py],'k','LineWidth',2)
% ball
% rectangle('Position',[px-mr/2,py-mr/2,mr,mr],'Curvature',1,'FaceColor',[.1 0.1 1])

xlim([-5 5]);
ylim([-2 2.5]);
set(gcf,'Position',[10 55 1000 400])
drawnow
hold off