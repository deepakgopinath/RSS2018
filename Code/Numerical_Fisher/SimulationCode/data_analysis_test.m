clear all; clc; close all;

%%
spaceList = {'R22', 'R33', 'SE2', 'SE3'}; %R2, R3, SE2, SE3
condList = {'POT', 'ENT', 'FI', 'DISAMB'};
delta_t = 0.1;
total_subjects = 400;

total_time_all = zeros(total_subjects, length(condList), length(spaceList)); %by deciding how soon does the chatter begin?
initial_alpha = -999*ones(total_subjects, length(condList), length(spaceList)); %initialize the alpha time as big negative. If it stays like this it means that the assistance was never triggered. 
percentage_correct_inference = zeros(total_subjects, length(condList), length(spaceList));
percentage_alpha = zeros(total_subjects, length(condList), length(spaceList)); %for this the chattering has to be detected and snipped. 

final_dist_to_goal = -999*ones(total_subjects, length(condList), length(spaceList)); %for all spaces. 
final_angle_to_goal = -999*ones(total_subjects, length(condList), length(spaceList)); %only SE2 and SE3

ng_array = zeros(total_subjects, length(spaceList)); %number of goals for each conditions. 
intent_type_array = cell(total_subjects, length(spaceList));

for i=1:length(spaceList) %type of World
    dirname = strcat(spaceList{i}, '_DATA');
    fnames = dir(dirname);
    numfids = length(fnames);
    for index=3:3 + (total_subjects-1)  %50 samples from each WORLD. First two files in the directory are . and ..
        n = fnames(index).name;
        disp(n);
        load(n);
        k = index - 2;
%%  compute fraction of distance remaining to the goal. 
        if i == 1 || i == 2
            final_dist_to_goal(k, 1, i) = norm(traj_POT(:,end) - xg(:, random_goal_index))/norm(traj_POT(:, 1) - xg(:, random_goal_index));
            final_dist_to_goal(k, 2, i) = norm(traj_ENT(:,end) - xg(:, random_goal_index))/norm(traj_ENT(:, 1) - xg(:, random_goal_index));
            final_dist_to_goal(k, 3, i) = norm(traj_FI(:,end) - xg(:, random_goal_index))/norm(traj_FI(:, 1) - xg(:, random_goal_index));
            final_dist_to_goal(k, 4, i) = norm(traj_DISAMB(:,end) - xg(:, random_goal_index))/norm(traj_DISAMB(:,1) - xg(:, random_goal_index));
        end
        if i == 3 
            final_dist_to_goal(k, 1, i) = norm(traj_POT(1:2,end) - xg(1:2, random_goal_index))/norm(traj_POT(1:2,1) - xg(1:2, random_goal_index));
            final_dist_to_goal(k, 2, i) = norm(traj_ENT(1:2,end) - xg(1:2, random_goal_index))/norm(traj_ENT(1:2,1) - xg(1:2, random_goal_index));
            final_dist_to_goal(k, 3, i) = norm(traj_FI(1:2,end) - xg(1:2, random_goal_index))/norm(traj_FI(1:2,1) - xg(1:2, random_goal_index));
            final_dist_to_goal(k, 4, i) = norm(traj_DISAMB(1:2,end) - xg(1:2, random_goal_index))/norm(traj_DISAMB(1:2,1) - xg(1:2, random_goal_index));
            
            final_angle_to_goal(k, 1, i) = wrapToPi(abs(traj_POT(3, end) - xg(3, random_goal_index)))/wrapToPi(abs(traj_POT(3, 1) - xg(3, random_goal_index)));
            final_angle_to_goal(k, 2, i) = wrapToPi(abs(traj_ENT(3, end) - xg(3, random_goal_index)))/wrapToPi(abs(traj_ENT(3, 1) - xg(3, random_goal_index)));
            final_angle_to_goal(k, 3, i) = wrapToPi(abs(traj_FI(3, end) - xg(3, random_goal_index)))/wrapToPi(abs(traj_FI(3, 1) - xg(3, random_goal_index)));
            final_angle_to_goal(k, 4, i) = wrapToPi(abs(traj_DISAMB(3, end) - xg(3, random_goal_index)))/wrapToPi(abs(traj_DISAMB(3, 1) - xg(3, random_goal_index)));
        end
        if i == 4 %SE3 is dealt with differently as the trajectory contains the 4 by 4 homogenous matrix. 
            final_dist_to_goal(k, 1, i) = norm(traj_POT{end}(1:3,4) - xg_T(1:3,4,random_goal_index))/norm(traj_POT{1}(1:3,4) - xg_T(1:3,4,random_goal_index));
            final_dist_to_goal(k, 2, i) = norm(traj_ENT{end}(1:3,4) - xg_T(1:3,4,random_goal_index))/norm(traj_ENT{1}(1:3,4) - xg_T(1:3,4,random_goal_index));
            final_dist_to_goal(k, 3, i) = norm(traj_FI{end}(1:3,4) - xg_T(1:3,4,random_goal_index))/norm(traj_FI{1}(1:3,4) - xg_T(1:3,4,random_goal_index));
            final_dist_to_goal(k, 4, i) = norm(traj_DISAMB{end}(1:3,4) - xg_T(1:3,4,random_goal_index))/norm(traj_DISAMB{1}(1:3,4) - xg_T(1:3,4,random_goal_index));
            
            final_angle_to_goal(k, 1, i) = compute_diff_angle(traj_POT{end}, xg_T(:, :, random_goal_index))/compute_diff_angle(traj_POT{1}, xg_T(:, :, random_goal_index));
            final_angle_to_goal(k, 2, i) = compute_diff_angle(traj_ENT{end}, xg_T(:, :, random_goal_index))/compute_diff_angle(traj_ENT{1}, xg_T(:, :, random_goal_index));
            final_angle_to_goal(k, 3, i) = compute_diff_angle(traj_FI{end}, xg_T(:, :, random_goal_index))/compute_diff_angle(traj_FI{1}, xg_T(:, :, random_goal_index));
            final_angle_to_goal(k, 4, i) = compute_diff_angle(traj_DISAMB{end}, xg_T(:, :, random_goal_index))/compute_diff_angle(traj_DISAMB{1}, xg_T(:, :, random_goal_index));
        end
%% identify the percentage time the inference is correct.
        percentage_correct_inference(k, 1, i) = sum(curr_goal_POT == random_goal_index)/length(curr_goal_POT);
        percentage_correct_inference(k, 2, i) = sum(curr_goal_ENT == random_goal_index)/length(curr_goal_ENT);
        percentage_correct_inference(k, 3, i) = sum(curr_goal_FI == random_goal_index)/length(curr_goal_FI);
        percentage_correct_inference(k, 4, i) = sum(curr_goal_DISAMB == random_goal_index)/length(curr_goal_DISAMB);
%% identify percentage time the assistance is active. 
        percentage_alpha(k, 1, i) = sum(alpha_POT > 0.0)/length(alpha_POT);
        percentage_alpha(k, 2, i) = sum(alpha_ENT > 0.0)/length(alpha_ENT);
        percentage_alpha(k, 3, i) = sum(alpha_FI > 0.0)/length(alpha_FI);
        percentage_alpha(k, 4, i) = sum(alpha_DISAMB > 0.0)/length(alpha_DISAMB);
        
        %percentage time the assistance is active towards the correct goal
%% number of goals in each trial
        ng_array(k, i) = ng;
        intent_type_array{k, i} = intent_type;
%% compute initial alpha time stamp    
               % with considering whether the assistance was towards the "human's
        % intended goal
%         if any((curr_goal_POT == random_goal_index).*alpha_POT)
%             initial_alpha(k, 1, i) = find((curr_goal_POT == random_goal_index).*alpha_POT > 0.0, 1)*delta_t; %time stamp at which alpha becomes non-zero first. 
%         end
%         if  any((curr_goal_ENT == random_goal_index).*alpha_ENT)
%             initial_alpha(k, 2, i) = find((curr_goal_ENT == random_goal_index).*alpha_ENT > 0.0, 1)*delta_t;
%         end
%         if  any((curr_goal_FI == random_goal_index).*alpha_FI)
%             initial_alpha(k, 3, i) = find((curr_goal_FI == random_goal_index).*alpha_FI > 0.0, 1)*delta_t;
%         end
%         if any((curr_goal_DISAMB == random_goal_index).*alpha_DISAMB)
%             initial_alpha(k, 4, i) = find((curr_goal_DISAMB == random_goal_index).*alpha_DISAMB > 0.0, 1)*delta_t;
%         end
       

% without considering whether the assistance was towards the "human's
% intended goal
        if any(alpha_POT)
            initial_alpha(k, 1, i) = find(alpha_POT > 0.0, 1)*delta_t;
%             percentage_alpha(k, 1, i) = 
        end
        if any(alpha_ENT)
            initial_alpha(k, 2, i) = find(alpha_ENT > 0.0, 1)*delta_t;
        end
        if any(alpha_FI)
            initial_alpha(k, 3, i) = find(alpha_FI > 0.0, 1)*delta_t;
        end
        if any(alpha_DISAMB)
            initial_alpha(k, 4, i) = find(alpha_DISAMB > 0.0, 1)*delta_t;
        end
    end
end

%%
R2_IA = initial_alpha(:,:,3);

bins = 15;
col1 = R2_IA(:, 1); %POT IA
col1(col1 == -999) = []; %remove trials where the assistance never kicked in. 
histogram(col1, bins);

figure;
col2 = R2_IA(:, 2); %ENT IA
col2(col2 == -999) = [];
histogram(col2, bins);

figure;
col3 = R2_IA(:, 3); %FI IA
col3(col3 == -999) = [];
histogram(col3, bins);

figure;
col4 = R2_IA(:, 4); %DISAMB IA
col4(col4 == -999) = [];
histogram(col4, bins);

%% 
R2_DTG = final_dist_to_goal(:,:,1);
bins = 20;
col1 = R2_DTG(:, 1); %POT IA
col1(col1 == -999) = []; %remove trials where the assistance never kicked in. 
histogram(col1, bins);

figure;
col2 = R2_DTG(:, 2); %ENT IA
col2(col2 == -999) = [];
histogram(col2, bins);

figure;
col3 = R2_DTG(:, 3); %FI IA
col3(col3 == -999) = [];
histogram(col3, bins);

figure;
col4 = R2_DTG(:, 4); %FI IA
col4(col4 == -999) = [];
histogram(col4, bins);

%%
function diff_angle = compute_diff_angle(Tr, Tg)
    Rg = Tg(1:3, 1:3); Rr = Tr(1:3, 1:3);
    Rdiff = Rg*(Rr^-1);
    diff_angaxis = MatrixLog3(Rdiff); %unnormalized
    [~, diff_angle] = AxisAng3(diff_angaxis);
end