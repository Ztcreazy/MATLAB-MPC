function dy = differential_mass(y,m,k,b,u)

dy(1,1) = y(2);
dy(2,1) = -k/m*y(1) - b/m*y(2) + u/m;
