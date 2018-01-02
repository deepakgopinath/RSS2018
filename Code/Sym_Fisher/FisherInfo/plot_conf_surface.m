
uh = [0,-1];
xg = [2,1];
[x_xr, y_xr] = meshgrid(-5:0.5:5, -5:1:5);

den = (sqrt((xg(1) - x_xr).^2 + (xg(2) - y_xr).^2))*(norm(uh));
num = uh(1).*(xg(1) - x_xr) + uh(2).*(xg(2) - y_xr);
costh = num./den;
cg = (1 + costh)./2;
surf(x_xr, y_xr, cg); hold on;
scatter3(xg(1), xg(2), 0, 100, 'filled', 'k');
xlabel('XPos');ylabel('YPos');zlabel('Confidence value');
quiver3(0,0, 0, uh(1), uh(2), 0,'LineWidth', 2, 'MaxHeadSize', 2);