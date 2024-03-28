function cost = Cost(stage,x,u,p)

w = eye(2);
cost = u'*w*u;