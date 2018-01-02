clear all; clc; close all;
%%
global num_modes ng nd xg xr delta_t;
ng = 2;
nd = 3;
num_modes = 3;
delta_t = 0.05;
xg = [0.45, -0.438, 0.23;
      -0.45, -0.330, 0.2;
      0.1, -0.45, 0.3]';
xr = [0.405, -0.128, 0.287]';
uh = 0.2*[0,0,1]';

%%
%PLOT GOALS AND ROBOT and UH
figure;
scatter3(xg(1,1:ng), xg(2,1:ng), xg(3,1:ng), 180, 'k', 'filled'); grid on; hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
scatter3(xr(1), xr(2), xr(3), 140, 'r', 'filled');
quiver3(xr(1), xr(2), xr(3), uh(1), uh(2), uh(3), 0, 'LineWidth', 2, 'MaxHeadSize', 2); %current robot velocity arrow
for i=1:ng %vectors connecting robot and goals.
    quiver3(xr(1), xr(2), xr(3), xg(1,i) - xr(1), xg(2,i) - xr(2), xg(3,i) - xr(3), 'LineWidth', 1.5, 'LineStyle', '-.');
end
xrange = 0.5*[-2,2]; %set axis limits
yrange = 0.5*[-2,2];
zrange = 0.5*[-2,2];
line(xrange, [0,0], [0,0], 'Color', 'k', 'LineWidth', 1.5); %draw x and y axes.
line([0,0], yrange, [0,0], 'Color', 'k','LineWidth', 1.5);
line([0,0], [0,0], zrange, 'Color', 'k','LineWidth', 1.5);
axis([xrange, yrange, zrange]);
axis square;
view([-142,31]);
%%
total_time = 1;
T = 0:delta_t:total_time;
traj = zeros(3, length(T));
traj(:, 1) = xr;
pgs = zeros(ng, length(T));
pgs(:, 1) = (1/ng)*ones(ng,1); %init pg
for i=1:length(T)-1
%     uh = 0.2*[1,0,0]';
    traj(:, i+1) = sim_dyn(traj(:, i), uh); %sim_dyn intenrally updates global robot state
%     traj(:, i+1) = traj(:, i);
    pgs(:, i+1) = compute_p_of_g_dft(uh, traj(:,i), pgs(:, i));
end
disp(pgs);