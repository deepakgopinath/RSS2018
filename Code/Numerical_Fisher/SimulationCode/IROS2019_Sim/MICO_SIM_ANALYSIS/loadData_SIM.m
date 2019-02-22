initial_alpha = zeros(trials_per_task, length(tasks), total_subjects); %first column is going to be reaching and second column is pouring. each slice is a subject. Each row is one trial for a task
percentage_correct_inference = zeros(trials_per_task, length(tasks), total_subjects);
percentage_alpha = zeros(trials_per_task, length(tasks), total_subjects);
total_time_all = zeros(trials_per_task, length(tasks), total_subjects);
%%
for i=1:total_subjects
    user = subList{i};
    subid = i;
    trialId = trialList(i);
    fnames = dir(user);
    numfids = length(fnames);
    for j=3:numfids
        n = fnames(j).name;
        load(n);
        temp = n; n(end-3:end) = [];
        if i < 10
            if length(n) == 9
                trialnum = str2double(n(end-1:end));
            else
                trialnum = str2double(n(end));
            end
            if strcmp(n(3:4), 'RE')
                ta = 1;
            elseif strcmp(n(3:4), 'PO')
                ta = 2;
            end
            if strcmp(task_order{subid}, n(3:4))
                intended_goal = ph1_trial_mat{trialnum, 4, subid};
                mode_switch_type = ph1_trial_mat{trialnum, 3, subid};
            else
                intended_goal = ph2_trial_mat{trialnum, 4, subid};
                mode_switch_type = ph2_trial_mat{trialnum, 3, subid};
            end
            
        else
            if length(n) == 10
                trialnum = str2double(n(end-1:end));
            else
                trialnum = str2double(n(end));
            end
            if strcmp(n(4:5), 'RE')
                ta = 1;
            elseif strcmp(n(4:5), 'PO')
                ta = 2;
            end
            if strcmp(task_order{subid}, n(4:5))
                intended_goal = ph1_trial_mat{trialnum, 4, subid};
                mode_switch_type = ph1_trial_mat{trialnum, 3, subid};
            else
                intended_goal = ph2_trial_mat{trialnum, 4, subid};
                mode_switch_type = ph2_trial_mat{trialnum, 3, subid};
            end
        end
        
        total_time_all(trialnum, ta, subid) = total_time;
        
        
        alpha(1, :) = []; %REMOVE ZEROS FROM FIRST ROW. 
        alpha(:, 2) = alpha(:, 2) - start_time;
        alpha = trim_data(alpha, false);
        
        current_goal(1, :) = [];
        current_goal(:, 2) = current_goal(:, 2) - start_time;
        current_goal = trim_data(current_goal, false);
        
        if length(disamb_start_times) > length(disamb_end_times) %This happens when the goal is reached dyuring the computation
            disamb_start_times(end) = [];
        end
        disamb_markers = [disamb_start_times, disamb_end_times];
        disamb_markers(1, :) = [];
        disamb_markers = disamb_markers - start_time;
        mode_switch_computation_time = disamb_markers(:, 2) - disamb_markers(:, 1);
        
        real_total_time = total_time - sum(mode_switch_computation_time);
        
        %remove time points that were recorded when mode_switch_computation
        %was happening
        for kk=1:size(disamb_markers, 1)
            cg_comp_inds = find(current_goal(:, 2) > disamb_markers(kk, 1) & current_goal(:,2) < disamb_markers(kk, 2));
            if ~isempty(cg_comp_inds)
                current_goal(cg_comp_inds, :) = [];
            end
            alpha_comp_inds = find(alpha(:, 2) > disamb_markers(kk, 1) & alpha(:,2) < disamb_markers(kk, 2));
            if ~isempty(alpha_comp_inds)
                alpha(alpha_comp_inds, :) = [];
            end
        end
        
        if length(alpha(:, 1)) ~= length(current_goal(:, 1))
            diff_el = length(alpha(:, 1)) - length(current_goal(:, 1));
            if diff_el > 0
                alpha(end-diff_el+1:end, :) = [];
            else
                current_goal(end-abs(diff_el)+1:end, :) = [];
            end
        end
        if strcmp(mode_switch_type, 'pot')
            time_out = 39;
        elseif strcmp(mode_switch_type, 'kld')
            time_out = 238;
        end
        if total_time_all(trialnum, ta, subid) < time_out %if the trial had failures, that if it timed out go to else block
            percentage_correct_inference(trialnum, ta, subid) = sum(current_goal(:, 1) == intended_goal)/length(current_goal(:, 1));
            percentage_alpha(trialnum, ta, subid) = sum((current_goal(:, 1) == intended_goal).*alpha(:, 1) > 0.0)/length(alpha(:, 1));
%             initial_alpha(trialnum, ta, subid) = find((current_goal(:, 1) == intended_goal).*alpha(:, 1) > 0.0, 1)/length(alpha(:, 1));
            
            if ~isempty(find((current_goal(:, 1) == intended_goal).*alpha(:, 1) > 0.0))
                initial_alpha(trialnum, ta, subid) = find((current_goal(:, 1) == intended_goal).*alpha(:, 1) > 0.0, 1)/length(alpha(:, 1));
            else
                initial_alpha(trialnum, ta, subid) = 1.0;
            end
        else
            percentage_correct_inference(trialnum, ta, subid) = -999;
            percentage_alpha(trialnum, ta, subid) = -999;
            initial_alpha(trialnum, ta, subid) = -999;
        end
    end
end

function [td] = trim_data(d, isrow)
    if isrow
        ts = d(end, :);
    else
        ts = d(:, end);
    end
    first_t = find(ts < 0);
    
    if ~isempty(first_t)
        first_t = first_t(end);
        if isrow()
            d(:, 1:first_t) = [];
        else
            d(1:first_t, :) = [];
        end
    end
    td = d;
    
end