clear all; close all; clc;
%%
spaceList = {'R22', 'R33', 'SE2', 'SE3'}; %R2, R3, SE2, SE3
condList = {'POT', 'ENT', 'KL', 'DISAMB'};
delta_t = 0.1;
total_subjects = 800;

%% types of data to be looked at

initial_alpha = -999*ones(total_subjects, length(condList), length(spaceList)); %initialize the alpha time as big negative. If it stays like this it means that the assistance was never triggered. 
trajectory_time = -999*ones(total_subjects, length(condList), length(spaceList));
percentage_correct_inference = -999*ones(total_subjects, length(condList), length(spaceList));
percentage_alpha = -999*ones(total_subjects, length(condList), length(spaceList)); %for this the chattering has to be detected and snipped. 
ng_array = -999*ones(total_subjects, length(spaceList)); %number of goals for each conditions. 
intent_type_array = -999*ones(total_subjects, length(spaceList));
num_mode_switches = -999*ones(total_subjects, length(condList), length(spaceList));
trajectory_length = -999*ones(total_subjects, length(condList), length(spaceList));
intent_type_dict = containers.Map({'dft', 'bayes', 'conf'}, [1,2,3]);
%%
for i=1:length(spaceList)
    dirname = strcat(spaceList{i}, '_DATA_IROS_EXP');
    fnames = dir(dirname);
    numfids = length(fnames);
    for index=3:3 + (total_subjects-1)
        n = fnames(index).name;
        disp(n);
        load(n);
        k = index - 2;
        %% compute trajectory time. And trim all data points. 
        if i < 4
            %make traj and pgs the same length as everything else. 119 time
            %steps. !!!!
            traj_POT(:, end) = []; traj_ENT(:, end) = []; traj_KL(:, end) = []; traj_DISAMB(:, end) = [];
            pgs_POT(:, end) = []; pgs_ENT(:, end) = []; pgs_KL(:, end) = []; pgs_DISAMB(:, end) = [];

            % trim trajectories properly. 
            traj_POT(:, ~any(traj_POT)) = []; l_POT = size(traj_POT, 2);
            traj_ENT(:, ~any(traj_ENT)) = []; l_ENT = size(traj_ENT, 2);
            traj_KL(:, ~any(traj_KL)) = []; l_KL = size(traj_KL, 2);
            traj_DISAMB(:, ~any(traj_DISAMB)) = []; l_DISAMB = size(traj_DISAMB, 2);
            

        elseif i == 4 %deal with SE3 separately because the trajectory points are cell arrays and not matrices
            %make traj and pgs same length as other ones.
            traj_POT(end) = []; traj_ENT(end) = []; traj_KL(end) = []; traj_DISAMB(end) = []; 
            pgs_POT(:, end) = []; pgs_ENT(:, end) = []; pgs_KL(:, end) = []; pgs_DISAMB(:, end) = [];
            
            traj_POT(cellfun(@isempty, traj_POT)) = []; l_POT = length(traj_POT);
            traj_ENT(cellfun(@isempty, traj_ENT)) = []; l_ENT = length(traj_ENT); 
            if l_ENT == 120
                l_ENT = 119; traj_ENT(end) = [];
            end
            traj_KL(cellfun(@isempty, traj_KL)) = []; l_KL = length(traj_KL);
            if l_KL == 120
                l_KL = 119; traj_KL(end) = [];
            end
            traj_DISAMB(cellfun(@isempty, traj_DISAMB)) = []; l_DISAMB = length(traj_DISAMB);
            if l_DISAMB == 120
                l_DISAMB = 119; traj_DISAMB(end) = [];
            end
            
        end
%         if ~strcmp(intent_type, 'conf')
%             continue
%         end
        
        
        if i < 3
            trans_nd = size(traj_POT, 1);
        elseif i==3
            trans_nd = 2;
        elseif i == 4
            trans_nd = 3;
        end
        if i < 4
            l_traj = 0.0;
            for traj_i=1:size(traj_POT, 2) - 1
               l_traj = l_traj + norm(traj_POT(1:trans_nd, traj_i+1) - traj_POT(1:trans_nd, traj_i));
            end
            trajectory_length(k, 1, i) = l_traj;

            l_traj = 0.0;
            for traj_i=1:size(traj_ENT, 2) - 1
               l_traj = l_traj + norm(traj_ENT(1:trans_nd, traj_i+1) - traj_ENT(1:trans_nd, traj_i));
            end
            trajectory_length(k, 2, i) = l_traj;

            l_traj = 0.0;
            for traj_i=1:size(traj_KL, 2) - 1
               l_traj = l_traj + norm(traj_KL(1:trans_nd, traj_i+1) - traj_KL(1:trans_nd, traj_i));
            end
            trajectory_length(k, 3, i) = l_traj;

            l_traj = 0.0;
            for traj_i=1:size(traj_DISAMB, 2) - 1
               l_traj = l_traj + norm(traj_DISAMB(1:trans_nd, traj_i+1) - traj_DISAMB(1:trans_nd, traj_i));
            end
            trajectory_length(k, 4, i) = l_traj;
        else
            l_traj = 0.0;
            for traj_i=1:length(traj_POT)-1
                x_next = traj_POT{i+1}(1:3, 4);
                x_curr = traj_POT{i}(1:3, 4);
                l_traj = l_traj + norm(x_next - x_curr);
            end
            trajectory_length(k, 1, i) = l_traj;
            
            l_traj = 0.0;
            for traj_i=1:length(traj_ENT)-1
                x_next = traj_ENT{i+1}(1:3, 4);
                x_curr = traj_ENT{i}(1:3, 4);
                l_traj = l_traj + norm(x_next - x_curr);
            end
            trajectory_length(k, 2, i) = l_traj;
            
            l_traj = 0.0;
            for traj_i=1:length(traj_KL)-1
                x_next = traj_KL{i+1}(1:3, 4);
                x_curr = traj_KL{i}(1:3, 4);
                l_traj = l_traj + norm(x_next - x_curr);
            end
            trajectory_length(k, 3, i) = l_traj;
            
            l_traj = 0.0;
            for traj_i=1:length(traj_DISAMB)-1
                x_next = traj_DISAMB{i+1}(1:3, 4);
                x_curr = traj_DISAMB{i}(1:3, 4);
                l_traj = l_traj + norm(x_next - x_curr);
            end
            trajectory_length(k, 4, i) = l_traj;
        end
        
        %%
        %trim all other relevant variables to the same length
        %probabilities
        pgs_POT = pgs_POT(:, 1:l_POT);
        pgs_ENT = pgs_ENT(:, 1:l_ENT);
        pgs_KL = pgs_KL(:, 1:l_KL);
        pgs_DISAMB = pgs_DISAMB(:, 1:l_DISAMB);

        %alpha_values
        alpha_POT = alpha_POT(1:l_POT); alpha_POT(end) = alpha_POT(end - 1); %copy last value to avoid 0. The exit condition did not take care of this
        alpha_ENT = alpha_ENT(1:l_ENT); alpha_ENT(end) = alpha_ENT(end - 1);
        alpha_KL = alpha_KL(1:l_KL); alpha_KL(end) = alpha_KL(end - 1);
        alpha_DISAMB = alpha_DISAMB(1:l_DISAMB); alpha_DISAMB(end) = alpha_DISAMB(end - 1);

        %human velocity, robot autonomy and blended velocity
        uh_POT = uh_POT(:, 1:l_POT); uh_POT(:, end) = uh_POT(:, end-1);
        uh_ENT = uh_ENT(:, 1:l_ENT); uh_ENT(:, end) = uh_ENT(:, end-1);
        uh_KL = uh_KL(:, 1:l_KL); uh_KL(:, end) = uh_KL(:, end-1);
        uh_DISAMB = uh_DISAMB(:, 1:l_DISAMB); uh_DISAMB(:, end) = uh_DISAMB(:, end-1);

        ur_POT = ur_POT(:, 1:l_POT); ur_POT(:, end) = ur_POT(:, end-1);
        ur_ENT = ur_ENT(:, 1:l_ENT); ur_ENT(:, end) = ur_ENT(:, end-1);
        ur_KL = ur_KL(:, 1:l_KL); ur_KL(:, end) = ur_KL(:, end-1);
        ur_DISAMB = ur_DISAMB(:, 1:l_DISAMB); ur_DISAMB(:, end) = ur_DISAMB(:, end-1);

        blend_vel_POT = blend_vel_POT(:, 1:l_POT); blend_vel_POT(:, end) = blend_vel_POT(:, end-1);
        blend_vel_ENT = blend_vel_ENT(:, 1:l_ENT); blend_vel_ENT(:, end) = blend_vel_ENT(:, end-1);
        blend_vel_KL = blend_vel_KL(:, 1:l_KL); blend_vel_KL(:, end) = blend_vel_KL(:, end-1);
        blend_vel_DISAMB = blend_vel_DISAMB(:, 1:l_DISAMB); blend_vel_DISAMB(:, end) = blend_vel_DISAMB(:, end-1);

        % current goal 
        curr_goal_POT = curr_goal_POT(1:l_POT); curr_goal_POT(end) = curr_goal_POT(end-1);
        curr_goal_ENT = curr_goal_ENT(1:l_ENT); curr_goal_ENT(end) = curr_goal_ENT(end-1);
        curr_goal_KL = curr_goal_KL(1:l_KL); curr_goal_KL(end) = curr_goal_KL(end-1);
        curr_goal_DISAMB = curr_goal_DISAMB(1:l_DISAMB); curr_goal_DISAMB(end) = curr_goal_DISAMB(end-1);

        % current mode list. The number indicate the ith mode. if -1
        % then the mode is not changed, 
        optimal_modes_POT = optimal_modes_POT(1:l_POT); optimal_modes_POT(end) = optimal_modes_POT(end-1);
        optimal_modes_POT(1) = 1; optimal_modes_POT(optimal_modes_POT == -1) = [];
        optimal_modes_ENT = optimal_modes_ENT(1:l_ENT); optimal_modes_ENT(end) = optimal_modes_ENT(end-1);
        optimal_modes_ENT(1) = 1; optimal_modes_ENT(optimal_modes_ENT == -1) = [];
        optimal_modes_KL = optimal_modes_KL(1:l_KL); optimal_modes_KL(end) = optimal_modes_KL(end-1);
        optimal_modes_KL(1) = 1; optimal_modes_KL(optimal_modes_KL == -1) = [];
        optimal_modes_DISAMB = optimal_modes_DISAMB(1:l_DISAMB); optimal_modes_DISAMB(end) = optimal_modes_DISAMB(end-1);
        optimal_modes_DISAMB(1) = 1; optimal_modes_DISAMB(optimal_modes_DISAMB == -1) = [];
        
        %% identify the percentage time the inference is correct.
        percentage_correct_inference(k, 1, i) = sum(curr_goal_POT == random_goal_index)/length(curr_goal_POT);
        percentage_correct_inference(k, 2, i) = sum(curr_goal_ENT == random_goal_index)/length(curr_goal_ENT);
        percentage_correct_inference(k, 3, i) = sum(curr_goal_KL == random_goal_index)/length(curr_goal_KL);
        percentage_correct_inference(k, 4, i) = sum(curr_goal_DISAMB == random_goal_index)/length(curr_goal_DISAMB);
        %% identify percentage time the assistance is active. 
        percentage_alpha(k, 1, i) = sum((curr_goal_POT == random_goal_index).*alpha_POT > 0.0)/length(alpha_POT);
        percentage_alpha(k, 2, i) = sum((curr_goal_ENT == random_goal_index).*alpha_ENT > 0.0)/length(alpha_ENT);
        percentage_alpha(k, 3, i) = sum((curr_goal_KL == random_goal_index).*alpha_KL > 0.0)/length(alpha_KL);
        percentage_alpha(k, 4, i) = sum((curr_goal_DISAMB == random_goal_index).*alpha_DISAMB > 0.0)/length(alpha_DISAMB);
        %% number of goals in each trial
        ng_array(k, i) = ng;
        intent_type_array(k, i) = intent_type_dict(intent_type);
        %% trajectory time
        %store trial time
        trajectory_time(k, 1, i) = l_POT*delta_t;
        trajectory_time(k, 2, i) = l_ENT*delta_t;
        trajectory_time(k, 3, i) = l_KL*delta_t;
        trajectory_time(k, 4, i) = l_DISAMB*delta_t;  
        %% number of mode switches.
        num_mode_switches(k, 1, i) = sum(~(abs(diff(optimal_modes_POT)) == 0));
        num_mode_switches(k, 2, i) = sum(~(abs(diff(optimal_modes_ENT)) == 0));
        num_mode_switches(k, 3, i) = sum(~(abs(diff(optimal_modes_KL)) == 0));
        num_mode_switches(k, 4, i) = sum(~(abs(diff(optimal_modes_DISAMB)) == 0));
        %% initial alpha
        %store time at which assistance gets triggered towards the CORRECT goal initially. 
        if any((curr_goal_POT == random_goal_index).*alpha_POT)
            initial_alpha(k, 1, i) = (find((curr_goal_POT == random_goal_index).*alpha_POT > 0.0, 1)*delta_t)/trajectory_time(k, 1, i); %time stamp at which alpha becomes non-zero first. 
        end
        if  any((curr_goal_ENT == random_goal_index).*alpha_ENT)
            initial_alpha(k, 2, i) = (find((curr_goal_ENT == random_goal_index).*alpha_ENT > 0.0, 1)*delta_t)/trajectory_time(k, 2, i);
        end
        if  any((curr_goal_KL == random_goal_index).*alpha_KL)
            initial_alpha(k, 3, i) = (find((curr_goal_KL == random_goal_index).*alpha_KL > 0.0, 1)*delta_t)/trajectory_time(k, 3, i);
        end
        if any((curr_goal_DISAMB == random_goal_index).*alpha_DISAMB)
            initial_alpha(k, 4, i) = (find((curr_goal_DISAMB == random_goal_index).*alpha_DISAMB > 0.0, 1)*delta_t)/trajectory_time(k, 4, i);
        end
    end
end
