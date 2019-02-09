% clear all; clc; close all;
%%
trials = 8; %num trails for each interface
num_sub = 32;
tasks = {'r', 'p'};
assis = {'pot', 'kld'};
interfaces = {'j', 'h'};
interfaces_array = {'j','h','j','h';
                    'h','j','h','j';
                    'j','h','h','j';
                    'h','j','j', 'h'};


tps = tpi*length(assis)*length(interfaces);
tpt = tpi*length(interfaces);

%% CREATE TASK ORDER. INDEPENDENTLY FOR 12 SUB. 
task_order_subs = cell(num_sub, length(tasks));
% 0 for t1t2 and 1 for t2t1. 
%Divide total number of subjects by 2. floor it.
half_sub = floor(num_sub/2);
if rand < 0.5
    task_order_list = [zeros(half_sub, 1); ones(num_sub - half_sub, 1)];
else
    task_order_list = [ones(half_sub, 1); zeros(num_sub - half_sub, 1)];
end
task_order_list = task_order_list(randperm(length(task_order_list)), 1);
for i=1:num_sub
    if task_order_list(i) == 0
        task_order_subs(i, :) = {'r', 'p'};
    else
        task_order_subs(i, :) = {'p', 'r'};
    end
end

%% CREATE INTERFACE ORDER
interface_order_subs = cell(size(interfaces_array, 2), num_sub);
interface_order_list = repmat((1:size(interfaces_array, 2))', floor(num_sub/size(interfaces_array, 2)), 1);
interface_order_list = interface_order_list(randperm(length(interface_order_list)), 1);
interface_order_list = [interface_order_list; randsample(size(interfaces_array, 2), num_sub - size(interfaces_array, 2)*floor(num_sub/size(interfaces_array, 2)))];
for i=1:num_sub
    index = interface_order_list(i);
    interface_order_subs(:, i) = interfaces_array(index, :)';
end

%% CREATE WITHOUT (1) AND ONDEMAND (2) COMBINATION WITH 3+5 RATIO.
% FOR EACH SUBJECT THERE WILL BE 4 (3+5) COMBINATION. 4 COMES FROM EACH
% TASK/INTERFACE COMBINATION. 
condition_list = [1,1,1,2,2,2,2,2]';

condition_order_subs = cell(tps, num_sub);
condition_order_list = zeros(tps, num_sub);
for i=1:num_sub
    for j=1:length(assis)*length(interfaces)
        cdl = condition_list(randperm(length(condition_list)), 1);
        cdl = cdl(randperm(length(cdl)), 1);
        condition_order_list((j-1)*trials + 1:j*trials ,i) = cdl;
    end
end
for i=1:num_sub
    for j=1:tps
        condition_order_subs{j, i} = assis{condition_order_list(j,i)};
    end
end

