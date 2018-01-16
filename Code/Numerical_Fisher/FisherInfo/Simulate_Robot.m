clear all; clc; close all;

%%
global num_modes ng nd xg xr sig delta_t;
ng = 2;
nd = 3;
num_modes = 3;
% xg = [rand(1,ng)*4 - 2;rand(1,ng)* 4- 2; rand(1,ng)*8 - 4] + rand(nd,1) - rand(nd,1); %random goal positions. These will be treated as fixed parameters.
% xg = [randsample(-1:1, ng); -rand(1,ng)*2; randsample(-1:1, ng)] + rand(nd,1)*0.2 - rand(nd,1)*0.2; %random goal positions. These will be treated as fixed parameters.
% xg = [2.345585086756016,  -0.654414913243983,   3.345585086756016;
%    0.068458668712525,  -0.931541331287475,   2.068458668712525;
%   -2.389203961866748,   1.610796038133252,  -3.389203961866748;];
% xg= [-3, 0, 0.01;
%     -1, 0.5, 0;
%     3, -0.2, 0.01;
%     1, 0.1, -0.01]';
xg = [0.45, -0.438, 0.23;
      -0.45, -0.330, 0.2;
      0.1, -0.45, 0.3]';
% xr = [0.2, -0.1, 0.3]';
xr = [0.405, -0.128, 0.287]';
uh = [-1,0,0]';
sig = 0.1;
%%

%PLOT GOALS AND ROBOT and UH
% figure;
% scatter3(xg(1,1:ng), xg(2,1:ng), xg(3,1:ng), 180, 'k', 'filled'); grid on; hold on;
% xlabel('X'); ylabel('Y'); zlabel('Z');
% scatter3(xr(1), xr(2), xr(3), 140, 'r', 'filled');
% quiver3(xr(1), xr(2), xr(3), uh(1), uh(2), uh(3), 0, 'LineWidth', 2, 'MaxHeadSize', 2); %current robot velocity arrow
% for i=1:ng %vectors connecting robot and goals.
%     quiver3(xr(1), xr(2), xr(3), xg(1,i) - xr(1), xg(2,i) - xr(2), xg(3,i) - xr(3), 'LineWidth', 1.5, 'LineStyle', '-.');
% end
% xrange = 0.5*[-2,2]; %set axis limits
% yrange = 0.5*[-2,2];
% zrange = 0.5*[-2,2];
% line(xrange, [0,0], [0,0], 'Color', 'k', 'LineWidth', 1.5); %draw x and y axes.
% line([0,0], yrange, [0,0], 'Color', 'k','LineWidth', 1.5);
% line([0,0], [0,0], zrange, 'Color', 'k','LineWidth', 1.5);
% axis([xrange, yrange, zrange]);
% axis square;
% view([142,31]);
%%
[u, t] = process_bag('uservel2.bag');
delta_t = mean(diff(t));
% delta_t = 0.01; %control rate. 
total_time = t(end);
T = 0:delta_t:total_time;
% xr = [0.2, -0.1, 0.3]';
xr = [0.405, -0.128, 0.287]'; %initial point of the robot.
traj = zeros(3, length(T));
traj(:, 1) = xr;
%%
%%SIMULATE DYNAMICS AND COMPUTE P(g).
pgs_DFT = zeros(ng, length(T));
pgs_DFT(:, 1) = (1/ng)*ones(ng,1); %init pg

pgs_BAYES = zeros(ng, length(T));
pgs_BAYES(:, 1) = (1/ng)*ones(ng,1); %init pg

pgs_CONF = zeros(ng, length(T));
pgs_CONF(:, 1) = (1/ng)*ones(ng,1); %init pg


optimal_assistance_t = 10;
optimal_assistance_ind = find(t <= optimal_assistance_t);
optimal_assistance_ind = optimal_assistance_ind(end);
hist_length = 1;
for i=1:length(T)-1
    if i == optimal_assistance_ind
        %trigger the FI based optimal mode computation. 
%         best_mode = compute_optimal_mode_FI(traj(:,i), 'conf', '1d', pgs(:, i));
%         disp(traj(:, i)); disp(pgs(:, i));
%         best_mode = compute_optimal_mode_FI(traj(:,i), 'conf', '1d', pgs(:,i));
%         disp(best_mode);
%         best_mode = compute_optimal_mode_FI(traj(:, i), 'bayes', '1d', pgs(:, i));
%         best_mode = compute_optimal_mode_FI(traj(:,i), 'conf', '2d', pgs(:, i));
%         best_mode = compute_optimal_mode_FI(traj(:,i), 'dft', '2d', pgs(:,i));
%         best_mode = compute_optimal_mode_FI(traj(:, i), 'bayes', '2d', pgs(:, i));
    end
    uh = u(:, i);
    traj(:, i+1) = sim_dyn(traj(:, i), uh); %sim_dyn intenrally updates global robot state
    pgs_DFT(:, i+1) = compute_p_of_g_dft(uh, traj(:,i), pgs_DFT(:, i));
    pgs_CONF(:, i+1) = compute_conf(uh, traj(:,i));
    pgs_BAYES(:, i+1) = compute_bayes(uh, traj(:, i), pgs_BAYES(:, i));
%     pgs(:, i+1) = compute_bayes_n_R3(u(:, max(1, i-hist_length+1):i), traj(:, i), pgs(:,i));
    xr = traj(:, i+1); 
    
end

%%
subplot(2,2,1);
step_size = 20;
scatter3(traj(1, 1:step_size:end),traj(2, 1:step_size:end),traj(3, 1:step_size:end), 'b', 'filled'); hold on;

% scatter3(traj(1, optimal_assistance_ind),traj(2, optimal_assistance_ind),traj(3, optimal_assistance_ind),100, 'g', 'filled' );
scatter3(xg(1,1), xg(2,1), xg(3,1), 450, 'k', 'filled'); grid on; hold on;
scatter3(xg(1,2), xg(2,2), xg(3,2), 450, 'm', 'filled'); grid on; hold on;

xlabel('\bf X (m)'); ylabel('\bf Y (m)'); zlabel('\bf Z (m)');
xrange = 0.4*[-2,2]; %set axis limits
yrange = 0.4*[-2,2];
zrange = 0.4*[-2,2];
line(xrange, [0,0], [0,0], 'Color', 'k', 'LineWidth', 1.5); %draw x and y axes.
line([0,0], yrange, [0,0], 'Color', 'k','LineWidth', 1.5);
line([0,0], [0,0], zrange, 'Color', 'k','LineWidth', 1.5);
axis([xrange, yrange, zrange]);
scatter3(traj(1, 1),traj(2, 1),traj(3, 1), 100, 'r', 'filled');
scatter3(traj(1, end),traj(2, end),traj(3, end),100, 'g', 'filled' );
% legend({'Start', 'End'});
% axis square;
view([-222,31]);

%%
%plot control signal
subplot(2,2,2);
plot(u(1,:)/max(u(1,:)), 'r', 'LineWidth',1.5); hold on; 
plot(u(2,:)/max(u(2,:)), 'g', 'LineWidth',1.5);
plot(u(3,:)/max(u(2,:)), 'b', 'LineWidth',1.5);
legend({'x', 'y', 'z'});
xlabel('\bf Time Steps');
ylabel('\bf Velocity m/s');

%%
%plot pg
%%
%%
subplot(2,2,3);
colors = {'k', 'm', 'y'};
for i=1:ng
    plot(pgs_CONF(i, :), colors{i}, 'LineWidth', 1.5); hold on;
end
grid on;
xlabel('\bf Time Steps');
ylabel('\bf P(g)');
legend({'Goal L', 'Goal R'});
title('Probability Evolution - CONF');
ylim([0.0,1.0]);
% %%
% subplot(1,3,2);
% colors = {'k', 'm', 'y'};
% for i=1:ng
%     plot(pgs_BAYES(i, :), colors{i}, 'LineWidth', 1.5); hold on;
% end
% grid on;
% xlabel('\bf Time Steps');
% ylabel('\bf P(g)');
% legend({'Goal L', 'Goal R'});
% title('Probability Evolution - BAYES');
% ylim([0.0,1.0]);

%%
subplot(2,2,4);
colors = {'k', 'm', 'y'};
for i=1:ng
    plot(pgs_DFT(i, :), colors{i}, 'LineWidth', 1.5); hold on;
end
grid on;
xlabel('\bf Time Steps');
ylabel('\bf P(g)');
legend({'Goal L', 'Goal R'});
title('Probability Evolution - DFT');
ylim([0.0,1.0]);
