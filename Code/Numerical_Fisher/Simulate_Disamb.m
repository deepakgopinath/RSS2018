clear all; clc; close all;
%%
global ng delta_t xg;
num_dof = 3;
ng = 3;
nd = 3;
nm = 3; %2-D interface in 3D world
% cm = {[1,2], [3]};
cm = {[1],[2],[3]};
delta_t = 0.05;
% w1 = 0.5;
% w2 = 0.5;

% %CONFIDENCE ARRAYS 
% conf_p = zeros(ng, 1);
% conf_n = zeros(ng, 1);
% conf_pp = zeros(ng, 1);
% conf_nn = zeros(ng, 1);
% 
% Dkjp_c1 = zeros(nd, 1);
% Dkjn_c1 = zeros(nd, 1);
% Dkjp_c2 = zeros(nd, 1);
% Dkjn_c2 = zeros(nd, 1);
% 
% Dkjp = zeros(nd, 1);
% Dkjn = zeros(nd, 1);
% Dkj = zeros(nd, 1);
% Dcm = zeros(nm, 1);
% kstar = 0;
% cmstar = 0;
maxR = 0.5;
minR = -0.5;
minR  + (maxR - minR)*rand(1, ng);
% xg = [minR  + (maxR - minR)*rand(1, ng); -rand(1, ng) - 0.02; 0.5*rand(1, ng) + 0.1] + rand(nd,1)*0.2 - rand(nd,1)*0.2; %random goal positions. These will be treated as fixed parameters.
xg = [2, -2, 1.8;
      -1.5, -1.1, 1;
      2, -0.4, 0.1;]';
pg0 = (1/ng)*ones(ng, 1);
pg = (1/ng)*ones(ng, 1);
project_time_list = [1,1,2,2]';
projected_pg_list = zeros(nd*ng, length(project_time_list));
% xr = [0.405, -0.128, 0.287]';
xr = [0,-1.4,0.7]';
%%
%PLOT GOALS AND ROBOT and UH
figure;
colors = {'y', 'r', 'b'};
for i=1:ng
    scatter3(xg(1,i), xg(2,i), xg(3,i), 180, colors{i}, 'filled'); grid on; hold on;
end
% scatter3(xg(1,1:ng), xg(2,1:ng), xg(3,1:ng), 180, 'k', 'filled'); grid on; hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
scatter3(xr(1), xr(2), xr(3), 140, 'k', 'filled');
% quiver3(xr(1), xr(2), xr(3), uh(1), uh(2), uh(3), 0, 'LineWidth', 2, 'MaxHeadSize', 2); %current robot velocity arrow
for i=1:ng %vectors connecting robot and goals.
    quiver3(xr(1), xr(2), xr(3), xg(1,i) - xr(1), xg(2,i) - xr(2), xg(3,i) - xr(3), 'LineWidth', 1.5, 'LineStyle', '-.');
end
xrange = 0.8*[-4,4]; %set axis limits
yrange = 0.8*[-4,4];
zrange = 0.8*[-4,4];
line(xrange, [0,0], [0,0], 'Color', 'k', 'LineWidth', 1.5); %draw x and y axes.
line([0,0], yrange, [0,0], 'Color', 'k','LineWidth', 1.5);
line([0,0], [0,0], zrange, 'Color', 'k','LineWidth', 1.5);
axis([xrange, yrange, zrange]);
axis square;
view([210,21]);
%%
total_time = 2;
T = 0:delta_t:total_time;
cell_pgs = cell(nm, 1); %for each mode
cell_trajs = cell(nm, 1); %to collect trajectories for each. 
pgs = zeros(ng, length(T), ng); %probabilities for each goal (row), at each time step (column) for each uh towards each goal(3rd)
traj = zeros(nd, length(T), ng);
a = 0.5*rand(ng-1, 1); a = [1-sum(a); a];
for i=1:ng
    pgs(:, 1, i) = (1/ng)*ones(ng, 1); %initialize pg0
%     pgs(:, 1, i) = a;
    traj(:, 1, i) = xr;
end
%%

for i=1:nm %for each mode
    for j=1:ng %for control commands towards each goal
        uh = xg(:, j) - xr;
        zero_dim = setdiff(1:nd,cm{i});
        for jj=1:length(zero_dim)
            uh(zero_dim(jj)) = 0;
        end
        uh = 0.2*(uh/(norm(uh) + realmin));
        for k=1:length(T)-1
             traj(:, k+1, j) = sim_dyn(traj(:, k, j), uh); %sim_dyn intenrally updates global robot state
             pgs(:, k+1, j) = compute_p_of_g_dft(uh, traj(:,k,j), pgs(:, k, j));
        end
    end
    cell_pgs{i} = pgs;
    cell_trajs{i} = traj;
end

%% plot trajectories
% for i=1:nm
%     for ii = 1:ng
%         traj = cell_trajs{i}(:,:,ii);
%         figure;
%         for j=1:length(T)
%             scatter3(traj(1, j), traj(2, j), traj(3, j), 'b', 'filled'); hold on;
%         end
%         xrange = [-1,1]; %set axis limits
%         yrange = [-1,1];
%         zrange = [-1,1];
%         line(xrange, [0,0], [0,0], 'Color', 'k', 'LineWidth', 1.5); %draw x and y axes.
%         line([0,0], yrange, [0,0], 'Color', 'k','LineWidth', 1.5);
%         line([0,0], [0,0], zrange, 'Color', 'k','LineWidth', 1.5);
%         axis([xrange, yrange, zrange]);
%         axis square;
%         view([210,21]);
%     end
% end
%%
figure;
for i=1:nm %number of modes, in columns
    curr_cell = cell_pgs{i};
    for j=1:ng %to each goal, in rows
        pgs_data = curr_cell(:,:, j);
        subplot(ng, nm, (j-1)*nm  + 1 + i - 1);
        plot(T, pgs_data', 'LineWidth', 2); grid on; ylim([0,1.0]);
        if i == 1
            ylabel({strcat('Motion to Goal ', num2str(j)), ' P(g)'} );
        else
            ylabel('P(g)');
        end
        
        xlabel('Time');
        if j == 1
            title(strcat('Control mode ', num2str(i)));
        end
        
    end
end
%% DISAMB MOTIVATION PLOTS. 
%plot goals
% close all;
figure
xrange = 0.6*[-4,4]; %set axis limits
yrange = 0.6*[-4,0];
zrange = 0.6*[0,4];
colors = {[1,0.8,0], [1,0,0], [0,0,1]};
for ii=1:ng
    scatter3(xg(1,ii), xg(2,ii), xg(3,ii), 250, colors{ii}, 'filled'); grid on; hold on;
    for jj=1:nd
        target = xg(:, ii); 
        if jj==1 || jj==2
            target(jj) = -2.4;
        else
            target(jj) = 0;
        end
        line([xg(1,ii), target(1)],[xg(2,ii), target(2)],[xg(3,ii), target(3)],'Color', [0,0,0,0.6], 'LineWidth', 0.6, 'LineStyle', '-.');
    end
end

for jj=1:nd
    target = xr;
    if jj==1 || jj==2
        target(jj) = -2.4;
    else
        target(jj) = 0;
    end
    line([xr(1), target(1)],[xr(2), target(2)],[xr(3), target(3)],'Color', [0,0,0,0.6], 'LineWidth', 0.6, 'LineStyle', '-.');
    
end
xlabel('X'); ylabel('Y'); zlabel('Z');
scatter3(xr(1), xr(2), xr(3), 500, 'k', 'filled');

% line(xrange, [0,0], [0,0], 'Color', 'k', 'LineWidth', 1.5); %draw x and y axes.
% line([0,0], yrange, [0,0], 'Color', 'k','LineWidth', 1.5);
% line([0,0], [0,0], zrange, 'Color', 'k','LineWidth', 1.5);
axis([xrange, yrange, zrange]);
axis square;
% view([-221,26]);
view([117 11]);
tg = 3; %specify target goal. b,r,y = 1,2,3
color_g = {[1,0.8,0], [1,0,0], [0,0,1]};
for i=1:nm %along each axis
    pgs = cell_pgs{i}(:,:,tg); %probabilities for each axis/control dimension is present
    for j=1:ng %use j to offset the lines slightly. ng separate lines. 
        tc = color_g{j};
        sc = [0.95, 0.95, 0.95];
        sp = xr;
        if i==1
            if j==1
                max_prob =  max(pgs(1, :));
            elseif j == 3
                max_prob = max(pgs(3,:));
            else
                max_prob = 1;
            end
        elseif i==2 || i==3
            if j==3
                max_prob = max(pgs(3,:));
            end
        end
        for k=1:length(T)
            alpha = pgs(j,k);
            color_point = interpolate_color(sc, tc, alpha, max_prob);
            dis = abs(sp - xg(:, tg)); gap = 0.04;
            if i==1
                scatter3(sp(1) + k*dis(i)/length(T), sp(2), sp(3) + (j-1)*gap, 20, color_point, 'filled');
                scatter3(sp(1) + k*dis(i)/length(T) + dis(i)/(2*length(T)), sp(2), sp(3) + (j-1)*gap, 20, color_point, 'filled');
            
            end
            if i==2
                scatter3(sp(1), sp(2)+k*dis(i)/length(T), sp(3) + (j-1)*gap, 20, color_point, 'filled');
                scatter3(sp(1), sp(2)+k*dis(i)/length(T)+ dis(i)/(2*length(T)), sp(3) + (j-1)*gap, 20, color_point, 'filled');

            end
            if i==3
                scatter3(sp(1), sp(2) - (j-1)*gap, sp(3)-k*dis(i)/length(T), 20, color_point, 'filled');
                scatter3(sp(1), sp(2) - (j-1)*gap, sp(3)-k*dis(i)/length(T)-dis(i)/(2*length(T)), 20, color_point, 'filled');

            end
        end
    end
    %plot 3 lines along i axis.
end
% %%
% for i=1:nm %for each conttrol mode there will be different figure. 
%     figure;
%     curr_cell = cell_pgs{i};
%     for j=1:ng %motion towards each goal
%         pgs_data = curr_cell(:,:,j);
%         for k=1:ng
%             subplot(ng,ng, (k-1)*ng  + 1 + j - 1);
%             
%             %plot goals and robot position
%             for ii=1:ng
%                 scatter3(xg(1,ii), xg(2,i), xg(3,ii), 180, colors{ii}, 'filled'); grid on; hold on;
%             end
%             xlabel('X'); ylabel('Y'); zlabel('Z');
%             scatter3(xr(1), xr(2), xr(3), 140, 'k', 'filled');
%             line(xrange, [0,0], [0,0], 'Color', 'k', 'LineWidth', 1.5); %draw x and y axes.
%             line([0,0], yrange, [0,0], 'Color', 'k','LineWidth', 1.5);
%             line([0,0], [0,0], zrange, 'Color', 'k','LineWidth', 1.5);
%             axis([xrange, yrange, zrange]);
%             axis square;
%             view([210,21]);
%             
%             
%             
%         end
%         
%     end
% end

%%

function color = interpolate_color(start_color, target_color, alpha, max_prob)
%         if alpha < 0.4
%             alpha = alpha - 0.1;
%         end
        
     color = (1.0 - alpha/max_prob)*start_color + (alpha/max_prob)*target_color;
end
%%
% min_ws = -0.5;
% max_ws = 0.5;
% num_steps = 8;
% step_size = (max_ws - min_ws)/num_steps;
% ax_p = (min_ws:step_size:max_ws)' +0.001;
% [X,Y,Z] = meshgrid(ax_p);
% ws_points = [X(:) Y(:) Z(:)];
% disamb_modes = zeros(size(ws_points, 1), 1);
% % xr = ws_points(i, :)';
