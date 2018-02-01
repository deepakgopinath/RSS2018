%% THIS SCRIPT IS RESPONSIBLE FOR LOADING UP ALL DATA FROM THE MAT FILES, USING loadData.m AND STORING THE DATA IN APPROPRIATE
%% MATRICES ACCORDING TO INTERFACE/CONDITION/TASK COMBINATION. 

clear all; clc; close all;

%%
subList = {'H1','H2', 'H3', 'H4', 'H5', 'H6', 'H7', 'H8', 'H9', 'H10', 'H11', 'H12', 'H13', 'H14', 'H15', 'H16'};
total_subjects = length(subList);
trialList = (1:16)';
interfaces = {'J2', 'HA'};
tasks = {'RE','PO'}; %REACHING AND POURING. 
assis = {'pot', 'kld'};
task_order = {'RE','RE','RE','RE', 'RE', 'PO','RE', 'PO', 'RE', 'RE', 'PO', 'PO', 'RE', 'PO', 'PO', 'PO'};%,'PO','RE','PO','PO','RE'}; %Phase 1 tasks FOR EACH SUBJECTS. 
trials_per_task = 16;
skip_list = {'H3', 'H4', 'H8', 'H9'};
load('trial_order_testing_32.mat'); % LOAD THE TRIAL ORDER. THIS IS FOR loadData TO CHECK FOR SOME CRITERION. 
loadData_SIM;

%%
%percent ALPHA VALUES FOR EACH TRIAL.
pa_re_jkld = [];
pa_po_jkld = [];
pa_re_jpot = [];
pa_po_jpot = [];

pa_re_hkld = [];
pa_po_hkld = [];
pa_re_hpot = [];
pa_po_hpot = [];

% initial alpha
ia_re_jkld = [];
ia_po_jkld = [];
ia_re_jpot = [];
ia_po_jpot = [];

ia_re_hkld = [];
ia_po_hkld = [];
ia_re_hpot = [];
ia_po_hpot = [];

%percentage correct inference. 
pc_re_jkld = [];
pc_po_jkld = [];
pc_re_jpot = [];
pc_po_jpot = [];

pc_re_hkld = [];
pc_po_hkld = [];
pc_re_hpot = [];
pc_po_hpot = [];

%success rate
su_re_jkld = [];
su_po_jkld = [];
su_re_jpot = [];
su_po_jpot = [];

su_re_hkld = [];
su_po_hkld = [];
su_re_hpot = [];
su_po_hpot = [];

%%
for i=1:total_subjects
    user = subList{i};
    trialId = trialList(i);
    if any(strcmp(skip_list, user))
        continue;
    end
    ph1_trial_list = ph1_trial_mat(:,:,trialId);
    ph2_trial_list = ph2_trial_mat(:,:,trialId);
    
    if strcmp(user, 'H6') %fix human errors
        ph1_trial_list{6, 5} = 2;
    end
    
    %index
    j_ph1_ind = find(strcmp(ph1_trial_list(:,2), 'j'));
    h_ph1_ind = find(strcmp(ph1_trial_list(:,2), 'h'));
    kld_ph1_ind = find(strcmp(ph1_trial_list(:,3), 'kld'));
    pot_ph1_ind = find(strcmp(ph1_trial_list(:,3), 'pot'));
   
    j_ph2_ind = find(strcmp(ph2_trial_list(:,2), 'j'));
    h_ph2_ind = find(strcmp(ph2_trial_list(:,2), 'h'));
    kld_ph2_ind = find(strcmp(ph2_trial_list(:,3), 'kld'));
    pot_ph2_ind = find(strcmp(ph2_trial_list(:,3), 'pot'));
    
    ph1_jkld_ind = intersect(j_ph1_ind, kld_ph1_ind);
    ph2_jkld_ind = intersect(j_ph2_ind, kld_ph2_ind);
    
    ph1_jpot_ind = intersect(j_ph1_ind, pot_ph1_ind);
    ph2_jpot_ind = intersect(j_ph2_ind, pot_ph2_ind);
    
    ph1_hkld_ind = intersect(h_ph1_ind, kld_ph1_ind);
    ph2_hkld_ind = intersect(h_ph2_ind, kld_ph2_ind);
    
    ph1_hpot_ind = intersect(h_ph1_ind, pot_ph1_ind);
    ph2_hpot_ind = intersect(h_ph2_ind, pot_ph2_ind);
    
    ph1id = 1; ph2id = 2; 
    subid = i;
    
    if strcmp(task_order{i}, 'RE') %means phase 1 is RE
        pa_re_jkld = [pa_re_jkld; percentage_alpha(ph1_jkld_ind, 1, subid)];
        pa_re_jpot = [pa_re_jpot; percentage_alpha(ph1_jpot_ind, 1, subid)];
        pa_re_hkld = [pa_re_hkld; percentage_alpha(ph1_hkld_ind, 1, subid)];
        pa_re_hpot = [pa_re_hpot; percentage_alpha(ph1_hpot_ind, 1, subid)];
        
        pc_re_jkld = [pc_re_jkld; percentage_correct_inference(ph1_jkld_ind, 1, subid)];
        pc_re_jpot = [pc_re_jpot; percentage_correct_inference(ph1_jpot_ind, 1, subid)];
        pc_re_hkld = [pc_re_hkld; percentage_correct_inference(ph1_hkld_ind, 1, subid)];
        pc_re_hpot = [pc_re_hpot; percentage_correct_inference(ph1_hpot_ind, 1, subid)];
        
        ia_re_jkld = [ia_re_jkld; initial_alpha(ph1_jkld_ind, 1, subid)];
        ia_re_jpot = [ia_re_jpot; initial_alpha(ph1_jpot_ind, 1, subid)];
        ia_re_hkld = [ia_re_hkld; initial_alpha(ph1_hkld_ind, 1, subid)];
        ia_re_hpot = [ia_re_hpot; initial_alpha(ph1_hpot_ind, 1, subid)];
        
    else
        pa_re_jkld = [pa_re_jkld; percentage_alpha(ph2_jkld_ind, 1, subid)];
        pa_re_jpot = [pa_re_jpot; percentage_alpha(ph2_jpot_ind, 1, subid)];
        pa_re_hkld = [pa_re_hkld; percentage_alpha(ph2_hkld_ind, 1, subid)];
        pa_re_hpot = [pa_re_hpot; percentage_alpha(ph2_hpot_ind, 1, subid)];
        
        pc_re_jkld = [pc_re_jkld; percentage_correct_inference(ph2_jkld_ind, 1, subid)];
        pc_re_jpot = [pc_re_jpot; percentage_correct_inference(ph2_jpot_ind, 1, subid)];
        pc_re_hkld = [pc_re_hkld; percentage_correct_inference(ph2_hkld_ind, 1, subid)];
        pc_re_hpot = [pc_re_hpot; percentage_correct_inference(ph2_hpot_ind, 1, subid)];
        
        ia_re_jkld = [ia_re_jkld; initial_alpha(ph2_jkld_ind, 1, subid)];
        ia_re_jpot = [ia_re_jpot; initial_alpha(ph2_jpot_ind, 1, subid)];
        ia_re_hkld = [ia_re_hkld; initial_alpha(ph2_hkld_ind, 1, subid)];
        ia_re_hpot = [ia_re_hpot; initial_alpha(ph2_hpot_ind, 1, subid)];
    end
    
    if strcmp(task_order{i}, 'PO') %means phase 1 is RE
        pa_po_jkld = [pa_po_jkld; percentage_alpha(ph1_jkld_ind, 2, subid)];
        pa_po_jpot = [pa_po_jpot; percentage_alpha(ph1_jpot_ind, 2, subid)];
        pa_po_hkld = [pa_po_hkld; percentage_alpha(ph1_hkld_ind, 2, subid)];
        pa_po_hpot = [pa_po_hpot; percentage_alpha(ph1_hpot_ind, 2, subid)];
        
        pc_po_jkld = [pc_po_jkld; percentage_correct_inference(ph1_jkld_ind, 2, subid)];
        pc_po_jpot = [pc_po_jpot; percentage_correct_inference(ph1_jpot_ind, 2, subid)];
        pc_po_hkld = [pc_po_hkld; percentage_correct_inference(ph1_hkld_ind, 2, subid)];
        pc_po_hpot = [pc_po_hpot; percentage_correct_inference(ph1_hpot_ind, 2, subid)];
        
        ia_po_jkld = [ia_po_jkld; initial_alpha(ph1_jkld_ind, 2, subid)];
        ia_po_jpot = [ia_po_jpot; initial_alpha(ph1_jpot_ind, 2, subid)];
        ia_po_hkld = [ia_po_hkld; initial_alpha(ph1_hkld_ind, 2, subid)];
        ia_po_hpot = [ia_po_hpot; initial_alpha(ph1_hpot_ind, 2, subid)];
        
    else
        pa_po_jkld = [pa_po_jkld; percentage_alpha(ph2_jkld_ind, 2, subid)];
        pa_po_jpot = [pa_po_jpot; percentage_alpha(ph2_jpot_ind, 2, subid)];
        pa_po_hkld = [pa_po_hkld; percentage_alpha(ph2_hkld_ind, 2, subid)];
        pa_po_hpot = [pa_po_hpot; percentage_alpha(ph2_hpot_ind, 2, subid)];
        
        pc_po_jkld = [pc_po_jkld; percentage_correct_inference(ph2_jkld_ind, 2, subid)];
        pc_po_jpot = [pc_po_jpot; percentage_correct_inference(ph2_jpot_ind, 2, subid)];
        pc_po_hkld = [pc_po_hkld; percentage_correct_inference(ph2_hkld_ind, 2, subid)];
        pc_po_hpot = [pc_po_hpot; percentage_correct_inference(ph2_hpot_ind, 2, subid)];
        
        ia_po_jkld = [ia_po_jkld; initial_alpha(ph2_jkld_ind, 2, subid)];
        ia_po_jpot = [ia_po_jpot; initial_alpha(ph2_jpot_ind, 2, subid)];
        ia_po_hkld = [ia_po_hkld; initial_alpha(ph2_hkld_ind, 2, subid)];
        ia_po_hpot = [ia_po_hpot; initial_alpha(ph2_hpot_ind, 2, subid)];
    end
    
end


%%success_rate
 su_re_jpot = 1 - sum(ia_re_jpot < 0)/length(ia_re_jpot);
 su_re_jkld = 1 - sum(ia_re_jkld < 0)/length(ia_re_jkld);
 su_re_hpot = 1 - sum(ia_re_hpot < 0)/length(ia_re_hpot);
 su_re_hkld = 1 - sum(ia_re_hkld < 0)/length(ia_re_hkld);
 
 su_po_jpot = 1 - sum(ia_po_jpot < 0)/length(ia_po_jpot);
 su_po_jkld = 1 - sum(ia_po_jkld < 0)/length(ia_po_jkld);
 su_po_hpot = 1 - sum(ia_po_hpot < 0)/length(ia_po_hpot);
 su_po_hkld = 1 - sum(ia_po_hkld < 0)/length(ia_po_hkld);

%% only extract the successful trials. 

pa_re_jkld(pa_re_jkld <= 0) = [];
pa_po_jkld(pa_po_jkld <= 0) = [];
pa_re_jpot(pa_re_jpot <= 0) = [];
pa_po_jpot(pa_po_jpot <= 0) = [];

pa_re_hkld(pa_re_hkld <= 0) = [];
pa_po_hkld(pa_po_hkld <= 0) = [];
pa_re_hpot(pa_re_hpot <= 0) = [];
pa_po_hpot(pa_po_hpot <= 0) = [];

%initial alpha
ia_re_jkld(ia_re_jkld <= 0) = [];
ia_po_jkld(ia_po_jkld <= 0) = [];
ia_re_jpot(ia_re_jpot <= 0) = [];
ia_po_jpot(ia_po_jpot <= 0) = [];

ia_re_hkld(ia_re_hkld <= 0) = [];
ia_po_hkld(ia_po_hkld <= 0) = [];
ia_re_hpot(ia_re_hpot <= 0) = [];
ia_po_hpot(ia_po_hpot <= 0) = [];

%percentage correct inference. 
pc_re_jkld(pc_re_jkld <= 0) = [];
pc_po_jkld(pc_po_jkld <= 0) = [];
pc_re_jpot(pc_re_jpot <= 0) = [];
pc_po_jpot(pc_po_jpot <= 0) = [];

pc_re_hkld(pc_re_hkld <= 0) = [];
pc_po_hkld(pc_po_hkld <= 0) = [];
pc_re_hpot(pc_re_hpot <= 0) = [];
pc_po_hpot(pc_po_hpot <= 0) = [];
