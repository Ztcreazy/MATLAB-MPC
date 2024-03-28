function xdot = Dynamics(x,u,p)
    alpha = u(1);
    v = u(2);
    theta = x(3);
    beta = x(4);
    M = p(1);
    L1 = p(2);
    L2 = p(3);
    xdot(1,1) = v*cos(beta)*(1 + M/L1*tan(beta)*tan(alpha))*cos(theta);
    xdot(2,1) = v*cos(beta)*(1 + M/L1*tan(beta)*tan(alpha))*sin(theta);
    xdot(3,1) = v*(sin(beta)/L2 - M/(L1*L2)*cos(beta)*tan(alpha));
    xdot(4,1) = v*(tan(alpha)/L1 - sin(beta)/L2 + M/(L1*L2)*cos(beta)*tan(alpha));

end