num_sub = 32;
tpi_training = 7; %tel,tel, blending, button, button
task_training = {'t0'};
interfaces = {'j','h'};% 221144
% assis = {'tel', 'tel', 'blend', 'but', 'but', 'but', 'but'}; %tel*4, blend*2, button*8
assis = {'tel', 'tel', 'tel', 'tel', 'blend', 'blend', 'but', 'but','but','but','but','but','but','but'};
% goal_order = [1,1,1,1,2,1,2]';
goal_order = [1,1,1,1,1,1]'; % + 8 of random order of goals 1 or 2. 
tps_training = tpi_training*length(interfaces);
ph0_trial_mat = cell(tps_training,6,num_sub);

%%
%% CREATE TASK ORDER. INDEPENDENTLY FOR 12 SUB. 
interface_order_subs_tr = cell(num_sub, length(tasks));
% 0 for t1t2 and 1 for t2t1. 
%Divide total number of subjects by 2. floor it.
half_sub = floor(num_sub/2);
if rand < 0.5
    interface_order_list = [zeros(half_sub, 1); ones(num_sub - half_sub, 1)];
else
    interface_order_list = [ones(half_sub, 1); zeros(num_sub - half_sub, 1)];
end
interface_order_list = interface_order_list(randperm(length(interface_order_list)), 1);
for i=1:num_sub
    if interface_order_list(i) == 0
        interface_order_subs_tr(i, :) = {'j', 'h'};
    else
        interface_order_subs_tr(i, :) = {'h', 'j'};
    end
end

%% CREATE HOME INIT M COMBINATION. DEPENDS ON INTERFACE. FOR J2
total_trials_jtrain = tpi_training*num_sub;
num_modes_j = 5; %for joystick
num_home = 3;
total_combinations_jtrain = num_modes_j*num_home;
g_h_combinations_jtrain = allcomb(1:num_home, 1:num_modes_j);

combi_list_jtrain = repmat((1:total_combinations_jtrain)', floor(total_trials_jtrain/total_combinations_jtrain), 1);
combi_list_jtrain = combi_list_jtrain(randperm(length(combi_list_jtrain)), 1);
combi_list_jtrain = [combi_list_jtrain; randsample(total_combinations_jtrain, total_trials_jtrain - total_combinations_jtrain*floor(total_trials_jtrain/total_combinations_jtrain))];
combi_list_jtrain = reshape(combi_list_jtrain, tpi_training, num_sub);

%% CREATE HOME INIT M COMBINATION. DEPENDS ON INTERFACE. FOR HA
total_trials_hatrain = tpi_training*num_sub;
num_modes_ha = 7; %for headarray
num_home = 3;
total_combinations_hatrain = num_modes_ha*num_home;
g_h_combinations_hatrain = allcomb(1:num_home, 1:num_modes_ha);

combi_list_hatrain = repmat((1:total_combinations_hatrain)', floor(total_trials_hatrain/total_combinations_hatrain), 1);
combi_list_hatrain = combi_list_hatrain(randperm(length(combi_list_hatrain)), 1);
combi_list_hatrain = [combi_list_hatrain; randsample(total_combinations_hatrain, total_trials_hatrain - total_combinations_hatrain*floor(total_trials_hatrain/total_combinations_hatrain))];
combi_list_hatrain = reshape(combi_list_hatrain, tpi_training, num_sub);
%%

int_training_order_subs = cell(length(interfaces), num_sub);
int_training_order_list = repmat((1:length(interfaces))', floor(num_sub/length(interfaces)), 1);
int_training_order_list = int_training_order_list(randperm(length(int_training_order_list)), 1);
int_training_order_list = [int_training_order_list; randsample(length(interfaces), num_sub - length(interfaces)*floor(num_sub/length(interfaces)))];
for i=1:num_sub
    ph0_trial_mat(:,1,i) = repmat(task_training, tps_training, 1);
%     if int_training_order_list(i) == 1
%         ph0_trial_mat(1:tpi_training, 2, i) = repmat(interfaces(1), tpi_training, 1);
%         ph0_trial_mat(tpi_training+1:end, 2, i) = repmat(interfaces(2), tpi_training, 1);
%     else
%         ph0_trial_mat(1:tpi_training, 2, i) = repmat(interfaces(2), tpi_training, 1);
%         ph0_trial_mat(tpi_training+1:end, 2, i) = repmat(interfaces(1), tpi_training, 1);
%     end
    if sum(strcmp(interface_order_subs_tr(i, :), {'j', 'h'})) == 2
        ph0_trial_mat(:, 2, i) = {'j','j','h','h','j','h','j','j','j','j','h','h','h','h'};
    else
        ph0_trial_mat(:, 2, i) = {'h','h','j','j','h', 'j', 'h', 'h', 'h', 'h', 'j', 'j', 'j', 'j'};
    end
%     ph0_trial_mat(:, 3, i) = repmat(assis', 2, 1);
    ph0_trial_mat(:, 3, i) = assis;
    
    g_list = [ones(4,1); 2*ones(4, 1)];
    g_list = g_list(randperm(length(g_list)), 1);
    ph0_trial_mat(:, 4, i) = num2cell([goal_order; g_list]);
%     ph0_trial_mat(:, 4, i) = num2cell(repmat(goal_order, length(interfaces), 1));
    
    for j=1:length(interfaces)
        lb = (j-1)*tpi_training + 1;
        ub = j*tpi_training;
        if(strcmp(ph0_trial_mat(lb,2,i), 'j'))
            ind = combi_list_jtrain(:, i);
            ph0_trial_mat(lb:ub, 5, i) = num2cell(g_h_combinations_jtrain(ind, 1));
            ph0_trial_mat(lb:ub, 6, i) = num2cell(g_h_combinations_jtrain(ind, 2)-1);
        else
            ind = combi_list_hatrain(:, i);
            ph0_trial_mat(lb:ub, 5, i) = num2cell(g_h_combinations_hatrain(ind, 1));
            ph0_trial_mat(lb:ub, 6, i) = num2cell(g_h_combinations_hatrain(ind, 2)-1);
        end
    end
end
filename = strcat('trial_order_training_', num2str(num_sub), '.mat');
save(filename, 'ph0_trial_mat');