F = scatteredInterpolant
%the steps of tx and ty will allow you change smoothness
tx = min(X):max(X);
ty = mix(Y):max(Y);
[qx,qy] = meshgrid(tx,ty);
qz = F(qx,qy);
mesh(qx,qy,qz);
hold on;
plot3(x,y,z,'o');