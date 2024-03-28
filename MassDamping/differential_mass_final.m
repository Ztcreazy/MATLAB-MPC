function dy = differential_mass_final(y,m,k,b,deltau)

dy(1,1) = y(2);
dy(2,1) = -k/m*y(1) - b/m*y(2) + deltau/m + y(5)/m;
% i = i+1;
% if i == 50
%     y(4) = -0.2;
% elseif i == 100
%     y(4) = 0.2;
% end
dy(3,1) = y(4);
dy(4,1) = 0;
dy(5,1) = deltau;