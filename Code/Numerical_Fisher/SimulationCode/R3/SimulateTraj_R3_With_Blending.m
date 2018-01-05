% clear all; close all; clc;
%% 
%workspace limits
xrange = [-0.5, 0.5];
yrange = [-0.5, 0.5];
zrange = [-0.5, 0.5];

%global variables. 
global num_modes cm ng nd xg xr sig delta_t conf_thresh conf_max alpha_max;

%workspace params.
% ng = 3; %num of goals
max_ng = 6;
% ng = 4; %num of goals
ng = datasample(2:max_ng, 1); %spawn random number of goals. Maximum number is 6. At least 
nd = 3; %num of dimensions. by definition R^3
cm_options = {{1,2,3}, {[1,2], 3}};
max_nm = length(cm_options);
cm = cm_options{datasample(1:max_nm, 1)};
% cm = {1,2,3}; %{1,2,3}, %{[1,3], 2} %{[2,3], 1};
num_modes = length(cm); %

%spawn random goals and robot position. 
xg = [rand(1,ng)*range(xrange) + xrange(1); rand(1,ng)*range(yrange) + yrange(1); rand(1,ng)*range(zrange) + zrange(1)]; %random goal positions. These will be treated as fixed parameters.
xr = [rand(1,1)*range(xrange) + xrange(1); rand(1,1)*range(yrange) + yrange(1); rand(1,1)*range(zrange) + zrange(1)];
xr_true = xr;

%disamb parameters.
sig = 0.01; %For Fisher information
delta_t = 0.1; %For compute projections. 

%arbitration function parameters
conf_thresh = (1.2/ng);
conf_max = (1.4/ng);
alpha_max = 0.7;

%% plot the goals and robot
% figure;
% scatter3(xg(1,1:ng), xg(2,1:ng), xg(3,1:ng), 230, 'k', 'filled'); grid on; hold on;
% xlabel('X'); ylabel('Y'); zlabel('Z');
% scatter3(xr(1), xr(2), xr(3), 230, 'r', 'filled');
% offset = [-0.3, 0.3];
% line(xrange+offset, [0,0], [0,0], 'Color', 'r', 'LineWidth', 1.5); %draw x and y axes.
% line([0,0], yrange+offset, [0,0], 'Color', 'g','LineWidth', 1.5);
% line([0,0], [0,0], zrange+offset, 'Color', 'b','LineWidth', 1.5);
% axis([xrange+offset, yrange+offset, zrange+offset]);
% axis square;
% view([142,31]);

%%
%% Generate the random goal towards which the simulated human would move. 
random_goal_index = randsample(ng, 1);
random_goal = xg(:, random_goal_index);
% scatter3(random_goal(1), random_goal(2),random_goal(3), 230, 'm', 'filled'); grid on; hold on;

%%
intent_types = {'dft', 'conf', 'bayes'};
intent_type = intent_types{datasample(1:length(intent_types), 1)}; % or conf or bayes

%% BASELINE
total_time_steps = 120; %with delta_t of 0.1, this amounts to 10 seconds. We will assume that "mode switches" don't take time. 
pgs_POT = zeros(ng, total_time_steps);
optimal_modes_POT = zeros(total_time_steps-1, 1);
alpha_POT = zeros(total_time_steps-1, 1);
uh_POT = zeros(nd, total_time_steps-1);
ur_POT = zeros(nd, total_time_steps-1);
blend_vel_POT = zeros(nd, total_time_steps-1);
curr_goal_POT = zeros(total_time_steps-1, 1);
traj_POT = zeros(nd, total_time_steps);
traj_POT(:, 1) = xr;
pgs_POT(:, 1) = (1/ng)*ones(ng, 1);%uniform probability to start off with. This is the true pg distribution during the course of the trajectory
%internally projection
current_optimal_mode_POT = 1;
mode_comp_timesteps = 4;

for i=1:total_time_steps-1
    curr_goal_index_POT = datasample(find(pgs_POT(:, i) == max(pgs_POT(:, i))), 1);
    curr_goal_POT(i) = curr_goal_index_POT;
    %compute the optimal mode. store it away. 
    if mod(i-1, mode_comp_timesteps) == 0
        current_optimal_mode_POT_index = compute_optimal_mode_POT_R3(xg(:, curr_goal_index_POT), xr_true); 
        if length(current_optimal_mode_POT_index) > 1 %when there are equivalent modes. 
            current_optimal_mode_POT = cm{current_optimal_mode_POT_index(1)}; %pick the first one. 
            current_optimal_mode_POT_index = current_optimal_mode_POT_index(1); %update the index. 
        else
            if current_optimal_mode_POT_index ~= -1 %if -1, don't change any mode. keep previous node. So no need to update
                current_optimal_mode_POT = cm{current_optimal_mode_POT_index}; 
            end
        end
    end
    %gotta determine uh. Assumes human is executes the "straightest
    %possible motion in the current mode towards the specified goal". 
    uh = generate_full_uh(random_goal, xr);
    zero_dim = setdiff(1:nd,current_optimal_mode_POT);
    for jj=1:length(zero_dim)
        uh(zero_dim(jj)) = 0;
    end

    uh = 0.2*(uh./(abs(uh) + realmin));
    ur = generate_autonomy(curr_goal_index_POT); %autonomy command in full 2D space toward what it thinks is the current goal
    alpha_POT(i) = alpha_from_confidence(pgs_POT(curr_goal_index_POT, i)); %linear belnding param
    blend_vel = (1-alpha_POT(i))*uh + alpha_POT(i)*ur; %blended vel
    uh_POT(:, i) = uh; ur_POT(:, i) = ur; blend_vel_POT(:, i) = blend_vel;
    if strcmp(intent_type, 'dft')
        pgs_POT(:, i+1) = compute_p_of_g_dft_R3(uh, xr, pgs_POT(:, i));
    elseif strcmp(intent_type, 'bayes')
        pgs_POT(:, i+1) = compute_bayes_R3(uh, xr, pgs_POT(:, i));
    elseif strcmp(intent_type, 'conf')
        pgs_POT(:, i+1) = compute_conf_R3(uh, xr);
    end
    xr = sim_kinematics_R3(xr, blend_vel); %forward simulate kinematics with blending. 
    traj_POT(:, i+1) = xr;
    optimal_modes_POT(i) = current_optimal_mode_POT_index;
end

%% USING ENTROPY simulate motion towards random goal from starting xr

fprintf("SIMULATION USING ENTROPY INFORMATION \n");
xr = xr_true;
pgs_ENT = zeros(ng, total_time_steps);
optimal_modes_ENT = zeros(total_time_steps-1, 1);
curr_goal_ENT = zeros(total_time_steps-1, 1);
alpha_ENT = zeros(total_time_steps-1, 1);
uh_ENT = zeros(nd, total_time_steps-1);
ur_ENT = zeros(nd, total_time_steps-1);
blend_vel_ENT = zeros(nd, total_time_steps-1);
traj_ENT = zeros(nd, total_time_steps);
traj_ENT(:, 1) = xr; %(x,y,theta)
pgs_ENT(:, 1) = (1/ng)*ones(ng, 1);%uniform probability to start off with. This is the true pg distribution during the course of the trajectory
%internally projection

current_optimal_mode_ENT_index = 1;
current_optimal_mode_ENT = cm{current_optimal_mode_ENT_index};

for i=1:total_time_steps-1
    curr_goal_index_ENT = datasample(find(pgs_ENT(:, i) == max(pgs_ENT(:, i))), 1);
    curr_goal_ENT(i) = curr_goal_index_ENT;
    if mod(i-1, mode_comp_timesteps) == 0
        current_optimal_mode_ENT_index = compute_optimal_mode_ENT_R3(intent_type, xr, pgs_ENT(:, i)); 
        if length(current_optimal_mode_ENT_index) > 1 %when there are equivalent modes. 
            current_optimal_mode_ENT = cm{current_optimal_mode_ENT_index(1)}; %pick the first one. 
            current_optimal_mode_ENT_index = current_optimal_mode_ENT_index(1); %update the index. 
        else
            if current_optimal_mode_ENT_index ~= -1 %if -1, don't change any mode. keep previous node. So no need to update
                current_optimal_mode_ENT = cm{current_optimal_mode_ENT_index}; 
            end
        end
    end
    uh = generate_full_uh(random_goal, xr);
    zero_dim = setdiff(1:nd,current_optimal_mode_ENT);
    for jj=1:length(zero_dim)
        uh(zero_dim(jj)) = 0;
    end
    uh = 0.2*(uh./(abs(uh) + realmin)); %normalize
    ur = generate_autonomy(curr_goal_index_ENT);
    alpha_ENT(i) = alpha_from_confidence(pgs_ENT(curr_goal_index_ENT, i)); %linear belnding param
    blend_vel = (1-alpha_ENT(i))*uh + alpha_ENT(i)*ur; %blended vel
    uh_ENT(:, i) = uh; ur_ENT(:, i) = ur; blend_vel_ENT(:, i) = blend_vel;
    if strcmp(intent_type, 'dft')
        pgs_ENT(:, i+1) = compute_p_of_g_dft_R3(uh, xr, pgs_ENT(:, i));
    elseif strcmp(intent_type, 'bayes')
        pgs_ENT(:, i+1) = compute_bayes_R3(uh, xr, pgs_ENT(:, i));
    elseif strcmp(intent_type, 'conf')
        pgs_ENT(:, i+1) = compute_conf_R3(uh, xr);
    end
    xr = sim_kinematics_R3(xr, blend_vel);
    traj_ENT(:, i+1) = xr;
    optimal_modes_ENT(i) = current_optimal_mode_ENT_index; 
end

%% USING FI
fprintf("SIMULATION USING FISHER INFORMATION \n");
xr = xr_true;
pgs_FI = zeros(ng, total_time_steps);
optimal_modes_FI = zeros(total_time_steps-1, 1);
alpha_FI = zeros(total_time_steps-1, 1);
uh_FI = zeros(nd, total_time_steps-1);
ur_FI = zeros(nd, total_time_steps-1);
blend_vel_FI = zeros(nd, total_time_steps-1);
curr_goal_FI = zeros(total_time_steps-1, 1);
traj_FI = zeros(nd, total_time_steps);
traj_FI(:, 1) = xr; %(x,y,theta)
pgs_FI(:, 1) = (1/ng)*ones(ng, 1);%uniform probability to start off with. This is the true pg distribution during the course of the trajectory
%internally projection

current_optimal_mode_FI_index = 1;
current_optimal_mode_FI = cm{current_optimal_mode_FI_index};

for i=1:total_time_steps-1
    curr_goal_index_FI = datasample(find(pgs_FI(:, i) == max(pgs_FI(:, i))), 1);
    curr_goal_FI(i) =  curr_goal_index_FI;
    if mod(i-1, mode_comp_timesteps) == 0
        current_optimal_mode_FI_index = compute_optimal_mode_FI_R3(intent_type, xr, pgs_FI(:, i)); 
        if length(current_optimal_mode_FI_index) > 1 %when there are equivalent modes. 
            current_optimal_mode_FI = cm{current_optimal_mode_FI_index(1)}; %pick the first one. 
            current_optimal_mode_FI_index = current_optimal_mode_FI_index(1); %update the index. 
        else
            if current_optimal_mode_FI_index ~= -1 %if -1, don't change any mode. keep previous node. So no need to update
                current_optimal_mode_FI = cm{current_optimal_mode_FI_index}; 
            end
        end
    end
    uh = generate_full_uh(random_goal, xr);
    zero_dim = setdiff(1:nd,current_optimal_mode_FI);
    for jj=1:length(zero_dim)
        uh(zero_dim(jj)) = 0;
    end
    ur = generate_autonomy(curr_goal_index_FI);
    alpha_FI(i) = alpha_from_confidence(pgs_FI(curr_goal_index_FI, i)); %linear belnding param
    blend_vel = (1-alpha_FI(i))*uh + alpha_FI(i)*ur; %blended vel
    uh = 0.2*(uh./(abs(uh) + realmin));
    uh_FI(:, i) = uh; ur_FI(:, i) = ur; blend_vel_FI(:, i) = blend_vel;
    if strcmp(intent_type, 'dft')
        pgs_FI(:, i+1) = compute_p_of_g_dft_R3(uh, xr, pgs_FI(:, i));
    elseif strcmp(intent_type, 'bayes')
        pgs_FI(:, i+1) = compute_bayes_R3(uh, xr, pgs_FI(:, i));
    elseif strcmp(intent_type, 'conf')
        pgs_FI(:, i+1) = compute_conf_R3(uh, xr);
    end
    xr = sim_kinematics_R3(xr, blend_vel);
    traj_FI(:, i+1) = xr;
    optimal_modes_FI(i) = current_optimal_mode_FI_index; 
end

%% DISAMB METRIC BASED
fprintf("SIMULATION USING DISAMB METRIC \n");
xr = xr_true;
pgs_DISAMB = zeros(ng, total_time_steps);
optimal_modes_DISAMB = zeros(total_time_steps, 1);
alpha_DISAMB = zeros(total_time_steps-1, 1);
uh_DISAMB = zeros(nd, total_time_steps-1);
ur_DISAMB = zeros(nd, total_time_steps-1);
blend_vel_DISAMB = zeros(nd, total_time_steps-1);
curr_goal_DISAMB = zeros(total_time_steps-1, 1);
traj_DISAMB = zeros(nd, total_time_steps);
traj_DISAMB(:, 1) = xr; %(x,y,theta)
pgs_DISAMB(:, 1) = (1/ng)*ones(ng, 1);%uniform probability to start off with. This is the true pg distribution during the course of the trajectory
%internally projection

current_optimal_mode_DISAMB_index = 1;
current_optimal_mode_DISAMB = cm{current_optimal_mode_DISAMB_index};

for i=1:total_time_steps-1
    curr_goal_index_DISAMB = datasample(find(pgs_DISAMB(:, i) == max(pgs_DISAMB(:, i))), 1);
    curr_goal_DISAMB(i) = curr_goal_index_DISAMB;
    if mod(i-1, mode_comp_timesteps) == 0
        current_optimal_mode_DISAMB_index = compute_optimal_mode_DISAMB_R3(intent_type, xr, pgs_DISAMB(:, i)); 
        if length(current_optimal_mode_DISAMB_index) > 1 %when there are equivalent modes. 
            current_optimal_mode_DISAMB = cm{current_optimal_mode_DISAMB_index(1)}; %pick the first one. 
            current_optimal_mode_DISAMB_index = current_optimal_mode_DISAMB_index(1); %update the index. 
        else
            if current_optimal_mode_DISAMB_index ~= -1 %if -1, don't change any mode. keep previous node. So no need to update
                current_optimal_mode_DISAMB = cm{current_optimal_mode_DISAMB_index}; 
            end
        end
    end
    uh = generate_full_uh(random_goal, xr);
    zero_dim = setdiff(1:nd,current_optimal_mode_DISAMB);
    for jj=1:length(zero_dim)
        uh(zero_dim(jj)) = 0;
    end
    uh = 0.2*(uh./(abs(uh) + realmin));
    ur = generate_autonomy(curr_goal_index_DISAMB);
    alpha_DISAMB(i) = alpha_from_confidence(pgs_DISAMB(curr_goal_index_DISAMB, i)); %linear belnding param
    blend_vel = (1-alpha_DISAMB(i))*uh + alpha_DISAMB(i)*ur; %blended vel
    uh_DISAMB(:, i) = uh; ur_DISAMB(:, i) = ur; blend_vel_DISAMB(:, i) = blend_vel;
    if strcmp(intent_type, 'dft')
        pgs_DISAMB(:, i+1) = compute_p_of_g_dft_R3(uh, xr, pgs_DISAMB(:, i));
    elseif strcmp(intent_type, 'bayes')
        pgs_DISAMB(:, i+1) = compute_bayes_R3(uh, xr, pgs_DISAMB(:, i));
    elseif strcmp(intent_type, 'conf')
        pgs_DISAMB(:, i+1) = compute_conf_R3(uh, xr);
    end
    xr = sim_kinematics_R3(xr, blend_vel);
    traj_DISAMB(:, i+1) = xr;
    optimal_modes_DISAMB(i) = current_optimal_mode_DISAMB_index; 
end

%%
% hold on; 
% scatter3(traj_POT(1, :)', traj_POT(2, :)', traj_POT(3, :)', 'k', 'filled');
% scatter3(traj_ENT(1, :)', traj_ENT(2, :)', traj_ENT(3, :)', 'r', 'filled');
% scatter3(traj_FI(1, :)', traj_FI(2, :)',  traj_FI(3, :)', 'b', 'filled');
% scatter3(traj_DISAMB(1, :)', traj_DISAMB(2, :)',  traj_DISAMB(3, :)', 'g', 'filled');
% %%
% plot_script;
% plot_goal_match;
%%

%%
function uh = generate_full_uh(xg, xr) %full unnomralized uh
    global nd;
    uh = xg - xr;
    uh = uh + normrnd(0, 0.01, nd, 1);
end

%% simple potential field type of autonomy. 
function ur = generate_autonomy(goal_index)
    global xr xg;
    ur = xg(:, goal_index) - xr;
    if norm(ur) > 0.20
        ur = 0.2*ur/norm(ur);
    end
end
%% arbitration function

function alpha = alpha_from_confidence(goal_pg)
   global conf_thresh conf_max alpha_max
   if goal_pg <= conf_thresh
       alpha = 0.0;
   elseif goal_pg > conf_thresh && goal_pg <= conf_max
       alpha = (goal_pg - conf_thresh)*(alpha_max/(conf_max - conf_thresh));
   else
       alpha = alpha_max;
   end
end

%%
%%
function [ best_mode ] = compute_optimal_mode_POT_R3(xg, xr_true)
    global cm nd xr;
    Pk = zeros(nd, 1);
    Pcm = zeros(length(cm), 1); %Information density for each mode. 
    for i=1:nd
        Pk(i) = abs((xg(i) - xr(i))/(xg(i) - xr_true(i)));
    end
    for i=1:length(cm)
        Pcm(i) = sum(Pk(cm{i}));
    end
    best_mode = compute_best_mode(Pcm);
end