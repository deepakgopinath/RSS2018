clear all; close all; clc;
%% initial variables
global ng delta_t xgs_pos xgs_quat xgs_R xgs_T xr_pos xr_quat xr_R xr_T;
num_dof = 3;
ng = 3;
nd = 6;
nm = 4; %2-D interface in 3D world
cm = {[1,2], [1,3], [4,5], [6]};
% cm = {[1],[2],[3], [4], [5], [6]};
delta_t = 0.05;
% maxR = 0.5;
% minR = -0.5;
% % minR  + (maxR - minR)*rand(1, ng);
% xgs_pos = [minR  + (maxR - minR)*rand(1, ng); -rand(1, ng) - 0.02; 0.5*rand(1, ng) + 0.1] + rand(nd,1)*0.2 - rand(nd,1)*0.2; %random goal positions. These will be treated as fixed parameters.
xgs_pos = [2, -1, 1.8;
      2, -0.8, 0.1;
      -1.5, -1.2, 0.8]';
xgs_quat = zeros(4, ng);
xgs_R = zeros(3,3,ng);
xgs_T = zeros(4,4,ng);
%orientations for each of the goals. 
R = zeros(3,3,ng);
R(:,:,1)= [ 1 0 0;
        0 0 -1;
        0 1 0]; %flat
R(:,:,2) = [ 1 0 0;
        0 -1 0;
        0 0 -1]; %over the top
R(:,:,3) = [ 0 -1 0;
        0 0 -1;
        1 0 0]; %door knob
orient_list = [1,2,3];
for i=1:ng
    xgs_quat(:, i) = RToQuat(R(:,:, orient_list(i)));
    xgs_R(:,:,i) = R(:,:, orient_list(i));
    xgs_T(:,:,i) = [[xgs_R(:,:,i);[0,0,0]] [xgs_pos(:,i); 1]];
end

xr_pos = [0.5,-0.1,0.3]';
xr_R = [ 0 0 1;
        1 0 0;
        0 1 0];

xr_quat = RToQuat(xr_R);
xr_T = [[xr_R;[0,0,0]] [xr_pos; 1]];

%% PLOT GOALS AND ROBOT and UH
figure;
scatter3(xgs_pos(1,1:ng), xgs_pos(2,1:ng), xgs_pos(3,1:ng), 180, 'k', 'filled'); grid on; hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
scatter3(xr_pos(1), xr_pos(2), xr_pos(3), 140, 'r', 'filled');
% quiver3(xr_pos(1), xr_pos(2), xr_pos(3), uh(1), uh(2), uh(3), 0, 'LineWidth', 2, 'MaxHeadSize', 2); %current robot velocity arrow
for i=1:ng %vectors connecting robot and goals.
    quiver3(xr_pos(1), xr_pos(2), xr_pos(3), xgs_pos(1,i) - xr_pos(1), xgs_pos(2,i) - xr_pos(2), xgs_pos(3,i) - xr_pos(3), 'LineWidth', 1.5, 'LineStyle', '-.');
end
xrange = [-4,4]; %set axis limits
yrange = [-4,4];
zrange = [-4,4];
line(xrange, [0,0], [0,0], 'Color', 'k', 'LineWidth', 1.5); %draw x and y axes.
line([0,0], yrange, [0,0], 'Color', 'k','LineWidth', 1.5);
line([0,0], [0,0], zrange, 'Color', 'k','LineWidth', 1.5);
axis([xrange, yrange, zrange]);
axis square;
view([210,21]);

%% probability distribution related stuff. 
pg0 = (1/ng)*ones(ng, 1);
pg = (1/ng)*ones(ng, 1);

%% init probabilities and initial trajectory points. 
total_time = 2;
T = 0:delta_t:total_time;
cell_pgs = cell(nm, 1); %for each mode
cell_trajs = cell(nm, 1); %to collect trajectories for each mode
pgs = zeros(ng, length(T), ng); %probabilities for each goal (row), at each time step (column) for each uh towards each goal(3rd)
cell_ind_traj = cell(ng, 1);
for i=1:ng
   cell_ind_traj{i} = zeros(4,4,length(T));
end
a = 0.5*rand(ng-1, 1); a = [1-sum(a); a];
for i=1:ng
%     pgs(:, 1, i) = (1/ng)*ones(ng, 1); %initialize pg0
    pgs(:, 1, i) = a;
    cell_ind_traj{i}(:,:,1) = xr_T;
end

%% simulation of kinematics systematically. 

for i=1:nm %for each control mode
    for j=1:ng %for control commands towards each goal
        if sum(cm{i} >= 4) %rotation modes
            wbf = ones(3,1);
            zero_dim = setdiff(4:6,cm{i});
            for jj=1:length(zero_dim)
                wbf(zero_dim(jj) - 3) = 0;
            end
            disp(wbf);
        else %translation
%             uh = xgs_T(:, j) - xr_T(1:3, 4);
        end
        if sum(cm{i} >= 4)
            %simulate rotation
            
        end
        %simulate translation
        
%         for k=1:length(T)-1
%              traj(:, k+1, j) = sim_dyn(traj(:, k, j), uh); %sim_dyn intenrally updates global robot state
%              pgs(:, k+1, j) = compute_p_of_g_dft(uh, traj(:,k,j), pgs(:, k, j));
%         end
    end
%     cell_pgs{i} = pgs;
%     cell_trajs{i} = traj;
end
