ph1_trial_mat = cell(tpt, 6, num_sub); %task, interface, assistance, goal, init home, init mode
ph2_trial_mat = cell(tpt, 6, num_sub);

for i=1:num_sub
    %FILL TASK INFO FOR BOTH PHASES
    ph1_t = task_order_subs(i, 1);
    ph1_trial_mat(:,1,i) = repmat(ph1_t, tpt, 1);
    ph2_t = task_order_subs(i, 2);
    ph2_trial_mat(:,1,i) = repmat(ph2_t, tpt, 1);
    
    %INTERFACE INFO FOR BOTH PHASES
    ph1_i = interface_order_subs(1:2, i);
    ph1_trial_mat(1:tpi, 2, i) = repmat(ph1_i(1), tpi, 1);
    ph1_trial_mat(tpi+1:end, 2, i) = repmat(ph1_i(2), tpi, 1);
    ph2_i = interface_order_subs(3:4, i);
    ph2_trial_mat(1:tpi, 2, i) = repmat(ph2_i(1), tpi, 1);
    ph2_trial_mat(tpi+1:end, 2, i) = repmat(ph2_i(2), tpi, 1);
    
    %ASSISTANCE FOR BOTH PHASES
    ph1_c = condition_order_subs(1:tpt, i);
    ph1_trial_mat(:, 3, i) = ph1_c;
    ph2_c = condition_order_subs(tpt+1:end, i);
    ph2_trial_mat(:, 3, i) = ph2_c;
    
    %CHeck whether what interface/task combination is happened. occurs
    %every tpi times. Depending on that pick the appropriate matrix to grab
    %data from. 
    
    %FOR PHASE 1
    for j=1:length(interfaces)
        lb = (j-1)*tpi + 1;
        ub = j*tpi;
        if(strcmp(ph1_trial_mat(lb,1,i), 'r'))
            if(strcmp(ph1_trial_mat(lb,2,i), 'j'))
                ind = combi_list_rj(:, i);
                ph1_trial_mat(lb:ub, 4, i) = num2cell(g_h_sm_combinations_rj(ind, 1));
                ph1_trial_mat(lb:ub, 5, i) = num2cell(g_h_sm_combinations_rj(ind, 2));
                ph1_trial_mat(lb:ub, 6, i) = num2cell(g_h_sm_combinations_rj(ind, 3)-1);
            else
                ind = combi_list_rha(:, i);
                ph1_trial_mat(lb:ub, 4, i) = num2cell(g_h_sm_combinations_rha(ind, 1));
                ph1_trial_mat(lb:ub, 5, i) = num2cell(g_h_sm_combinations_rha(ind, 2));
                ph1_trial_mat(lb:ub, 6, i) = num2cell(g_h_sm_combinations_rha(ind, 3)-1);
            end
        elseif(strcmp(ph1_trial_mat(lb,1,i), 'p'))
            if(strcmp(ph1_trial_mat(lb,2,i), 'j'))
                ind = combi_list_pj(:, i);
                ph1_trial_mat(lb:ub, 4, i) = num2cell(g_h_sm_combinations_pj(ind, 1));
                ph1_trial_mat(lb:ub, 5, i) = num2cell(g_h_sm_combinations_pj(ind, 2));
                ph1_trial_mat(lb:ub, 6, i) = num2cell(g_h_sm_combinations_pj(ind, 3)-1);
            else
                ind = combi_list_pha(:, i);
                ph1_trial_mat(lb:ub, 4, i) = num2cell(g_h_sm_combinations_pha(ind, 1));
                ph1_trial_mat(lb:ub, 5, i) = num2cell(g_h_sm_combinations_pha(ind, 2));
                ph1_trial_mat(lb:ub, 6, i) = num2cell(g_h_sm_combinations_pha(ind, 3)-1);
            end
        end
    end
    
    %FOR PHASE 2
    for j=1:length(interfaces)
        lb = (j-1)*tpi + 1;
        ub = j*tpi;
        if(strcmp(ph2_trial_mat(lb,1,i), 'r'))
            if(strcmp(ph2_trial_mat(lb,2,i), 'j'))
                ind = combi_list_rj(:, i);
                ph2_trial_mat(lb:ub, 4, i) = num2cell(g_h_sm_combinations_rj(ind, 1));
                ph2_trial_mat(lb:ub, 5, i) = num2cell(g_h_sm_combinations_rj(ind, 2));
                ph2_trial_mat(lb:ub, 6, i) = num2cell(g_h_sm_combinations_rj(ind, 3)-1);
            else
                ind = combi_list_rha(:, i);
                ph2_trial_mat(lb:ub, 4, i) = num2cell(g_h_sm_combinations_rha(ind, 1));
                ph2_trial_mat(lb:ub, 5, i) = num2cell(g_h_sm_combinations_rha(ind, 2));
                ph2_trial_mat(lb:ub, 6, i) = num2cell(g_h_sm_combinations_rha(ind, 3)-1);
            end
        elseif(strcmp(ph2_trial_mat(lb,1,i), 'p'))
            if(strcmp(ph2_trial_mat(lb,2,i), 'j'))
                ind = combi_list_pj(:, i);
                ph2_trial_mat(lb:ub, 4, i) = num2cell(g_h_sm_combinations_pj(ind, 1));
                ph2_trial_mat(lb:ub, 5, i) = num2cell(g_h_sm_combinations_pj(ind, 2));
                ph2_trial_mat(lb:ub, 6, i) = num2cell(g_h_sm_combinations_pj(ind, 3)-1);
            else
                ind = combi_list_pha(:, i);
                ph2_trial_mat(lb:ub, 4, i) = num2cell(g_h_sm_combinations_pha(ind, 1));
                ph2_trial_mat(lb:ub, 5, i) = num2cell(g_h_sm_combinations_pha(ind, 2));
                ph2_trial_mat(lb:ub, 6, i) = num2cell(g_h_sm_combinations_pha(ind, 3)-1);
            end
        end
    end
end
filename = strcat('trial_order_testing_', num2str(num_sub), '.mat');
save(filename, 'ph1_trial_mat', 'ph2_trial_mat');