clear all; close all; clc;
%% REACHING TASK (5) WITH JOYSTICK (4) HOME(3). SCRIPT TO GENERATE AND DISTRIBUTE ALL POSSIBLE (GOAL, HOME, STARTING MODE) COMBO. 

tpi = 8; %num trails for each interface
num_sub = 32;
total_trials_rj = num_sub*tpi;
num_goal_r = 5; %for reaching
num_modes_j = 4; %for joystick
num_home = 3;
total_combinations_rj = num_goal_r*num_modes_j*num_home; %1:total_combinations have a unique map between the index and the (goal, home, starting mode) tuple.
g_h_sm_combinations_rj = allcomb(1:num_goal_r, 1:num_home, 1:num_modes_j);
%% GENERATE COMBO LIST FOR ALL SUBJECT R_J2
% combi_list_rj = repmat((1:total_combinations_rj)', floor(total_trials_rj/total_combinations_rj), 1);
% combi_list_rj = combi_list_rj(randperm(length(combi_list_rj)), 1);
% combi_list_rj = [combi_list_rj; randsample(total_combinations_rj, total_trials_rj -floor(total_trials_rj/total_combinations_rj)*total_combinations_rj)];
% 
combi_list_rj = repmat((1:total_combinations_rj)', floor(total_trials_rj/total_combinations_rj), 1);
combi_list_rj = combi_list_rj(randperm(length(combi_list_rj)), 1);
combi_list_rj = [combi_list_rj; randsample(total_combinations_rj, total_trials_rj -floor(total_trials_rj/total_combinations_rj)*total_combinations_rj)];
combi_list_rj = reshape(combi_list_rj, tpi, num_sub);
% 
% combi_list_rj = randperm(total_combinations_rj, total_trials_rj)';
% combi_list_rj = reshape(combi_list_rj, tpi, num_sub);
%% REACH TASK (5) WITH HA (6) HOME(3). SCRIPT TO GENERATE AND DISTRIBUTE ALL POSSIBLE (GOAL, HOME, STARTING MODE) COMBO.
tpi = 8; %num trails for each interface
% num_sub = 8;
total_trials_rha = num_sub*tpi;
num_goal_r = 5; %for reaching
num_modes_ha = 6; %for headarray
num_home = 3;
total_combinations_rha = num_goal_r*num_modes_ha*num_home; %1:total_combinations have a unique map between the index and the (goal, home, starting mode) tuple.
g_h_sm_combinations_rha = allcomb(1:num_goal_r, 1:num_home, 1:num_modes_ha);

%% GENERATE COMBO LIST FOR ALL SUBJECT R_hA
% Assume that total g_h_sm combinations for r_ha is higher than the total
% tpi. 
combi_list_rha = repmat((1:total_combinations_rha)', floor(total_trials_rha/total_combinations_rha), 1);
combi_list_rha = combi_list_rha(randperm(length(combi_list_rha)), 1);
combi_list_rha = [combi_list_rha; randsample(total_combinations_rha, total_trials_rha -floor(total_trials_rha/total_combinations_rha)*total_combinations_rha)];
combi_list_rha = reshape(combi_list_rha, tpi, num_sub);
%%
% combi_list_rha = randperm(total_combinations_rha, total_trials_rha)';% this has total_trials already. 
% combi_list_rha = reshape(combi_list_rha, tpi, num_sub);


%% *******************************************
%% POURING TASK (4) WITH JOYSTICK (5) HOME(3). SCRIPT TO GENERATE AND DISTRIBUTE ALL POSSIBLE (GOAL, HOME, STARTING MODE) COMBO. 
tpi = 8; %num trails for each interface
% num_sub = 8;
total_trials_pj = num_sub*tpi;
num_goal_p = 4; %for reaching
num_modes_j = 4; %for joystick
num_home = 3;
total_combinations_pj = num_goal_p*num_modes_j*num_home; %1:total_combinations have a unique map between the index and the (goal, home, starting mode) tuple.
g_h_sm_combinations_pj = allcomb(1:num_goal_p, 1:num_home, 1:num_modes_j);

%%
combi_list_pj = repmat((1:total_combinations_pj)', floor(total_trials_pj/total_combinations_pj), 1);
combi_list_pj = combi_list_pj(randperm(length(combi_list_pj)), 1);
combi_list_pj = [combi_list_pj; randsample(total_combinations_pj, total_trials_pj -floor(total_trials_pj/total_combinations_pj)*total_combinations_pj)];
combi_list_pj = reshape(combi_list_pj, tpi, num_sub);


%% POURING TASK (4) WITH HEADARRAY (7) HOME(3). SCRIPT TO GENERATE AND DISTRIBUTE ALL POSSIBLE (GOAL, HOME, STARTING MODE) COMBO. 
tpi = 8; %num trails for each interface
% num_sub = 8;
total_trials_pha = num_sub*tpi;
num_goal_p = 4; %for reaching
num_modes_ha = 6; %for joystick
num_home = 3;
total_combinations_pha = num_goal_p*num_modes_ha*num_home; %1:total_combinations have a unique map between the index and the (goal, home, starting mode) tuple.
g_h_sm_combinations_pha = allcomb(1:num_goal_p, 1:num_home, 1:num_modes_ha);

%% combinations < total tpi. same strategy 
combi_list_pha = repmat((1:total_combinations_pha)', floor(total_trials_pha/total_combinations_pha), 1);
combi_list_pha = combi_list_pha(randperm(length(combi_list_pha)), 1);
combi_list_pha = [combi_list_pha; randsample(total_combinations_pha, total_trials_pha -floor(total_trials_pha/total_combinations_pha)*total_combinations_pha)];
combi_list_pha = reshape(combi_list_pha, tpi, num_sub);
%%
% combi_list_pha = randperm(total_combinations_pha, total_trials_pha)';% this has total_trials already. 
% combi_list_pha = reshape(combi_list_pha, tpi, num_sub);
