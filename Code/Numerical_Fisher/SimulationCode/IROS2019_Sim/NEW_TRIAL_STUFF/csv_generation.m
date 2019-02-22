clear all; close all; clc;
%% LOAD TRIAL ORDER MAT FILES
num_sub = 32;
% filename_tr = strcat('trial_order_training_', num2str(num_sub), '.mat');
filename_te = strcat('trial_order_testing_', num2str(num_sub), '.mat');
% load(filename_tr);
load(filename_te);
%%
for i=1:num_sub
    fileID = fopen(strcat('H', int2str(i), '.txt'),'w');
    fprintf(fileID, 'Tasks \t Inter \t Assis \t Goal \t Home \t InitM\n');
%     fprintf(fileID, '\n');
    formatSpec = '%s\t\t %s\t\t %s\t\t %d\t\t %d\t\t %d\n';
%     %TRAINING TRIALS
%     t = ph0_trial_mat(:,:,i);
%     for j=1:size(t,1)
%         fprintf(fileID,formatSpec,t{j,:});
%     end
%     fprintf(fileID, '################################\n');
%     fprintf(fileID, '################################\n');
    %PHASE 1 TRIALS
    t = ph1_trial_mat(:,:,i);
    for j=1:size(t,1)
        fprintf(fileID,formatSpec,t{j,1}, t{j,2}, t{j,3}, t{j, 4:6});
    end
    %PHASE 2 TRIALS
    fprintf(fileID, '################################\n');
    fprintf(fileID, '################################\n');
    t = ph2_trial_mat(:,:,i);
    for j=1:size(t,1)
        fprintf(fileID,formatSpec,t{j,1}, t{j,2}, t{j,3}, t{j, 4:6});
    end
end
