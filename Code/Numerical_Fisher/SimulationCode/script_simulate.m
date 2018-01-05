clear all; clc; close all;

N = 400; %number of simulations
vars = {'alpha_', 'curr_goal_', 'optimal_modes_', 'pgs_', 'traj_', 'uh_', 'ur_', 'blend_vel_'};
types = {'POT', 'ENT', 'FI', 'DISAMB'};
%for R2 case
for ii=1:N
    %define global variables
   SimulateTraj_R2_With_Blending;
   filename = strcat('R22_DATA/DATAR22_', num2str(ii), '.mat');
   save(filename, 'ng', 'nd', 'cm', 'num_modes', 'xg', 'xr_true', 'intent_type','random_goal_index');
   for jj=1:length(vars)
        for kk=1:length(types)
            save(filename, strcat(vars{jj}, types{kk}), '-append');
        end
   end
   clear all; clc; close all;
   N = 400; %number of simulations
   vars = {'alpha_', 'curr_goal_', 'optimal_modes_', 'pgs_', 'traj_', 'uh_', 'ur_', 'blend_vel_'};
   types = {'POT', 'ENT', 'FI', 'DISAMB'};
end

%% R3 simulation.


vars = {'alpha_', 'curr_goal_', 'optimal_modes_', 'pgs_', 'traj_', 'uh_', 'ur_', 'blend_vel_'};
types = {'POT', 'ENT', 'FI', 'DISAMB'};
for ii=1:N
    %define global variables
   SimulateTraj_R3_With_Blending;
   filename = strcat('R33_DATA/DATAR33_', num2str(ii), '.mat');
   save(filename, 'ng', 'nd', 'cm', 'num_modes', 'xg', 'xr_true', 'intent_type','random_goal_index');
   for jj=1:length(vars)
        for kk=1:length(types)
            save(filename, strcat(vars{jj}, types{kk}), '-append');
        end
   end
   
   clear all; clc; close all;
   N = 400; %number of simulations
   vars = {'alpha_', 'curr_goal_', 'optimal_modes_', 'pgs_', 'traj_', 'uh_', 'ur_', 'blend_vel_'};
   types = {'POT', 'ENT', 'FI', 'DISAMB'};
end

%% SE2 SIMULATION

vars = {'alpha_', 'curr_goal_', 'optimal_modes_', 'pgs_', 'traj_', 'uh_', 'ur_', 'blend_vel_'};
types = {'POT', 'ENT', 'FI', 'DISAMB'};
for ii=1:N
    %define global variables
   SimulateTraj_SE2_With_Blending;
   filename = strcat('SE2_DATA/DATASE2_', num2str(ii), '.mat');
   save(filename, 'ng', 'nd', 'cm', 'num_modes', 'xg', 'xr_true', 'intent_type', 'random_goal_index');
   for jj=1:length(vars)
        for kk=1:length(types)
            save(filename, strcat(vars{jj}, types{kk}), '-append');
        end
   end
   
   clear all; clc; close all;
   N = 400; %number of simulations
   vars = {'alpha_', 'curr_goal_', 'optimal_modes_', 'pgs_', 'traj_', 'uh_', 'ur_', 'blend_vel_'};
   types = {'POT', 'ENT', 'FI', 'DISAMB'};
end

%% SE3 SIMULATION


vars = {'alpha_', 'curr_goal_', 'optimal_modes_', 'pgs_', 'traj_', 'uh_', 'ur_', 'blend_vel_'};
types = {'POT', 'ENT', 'FI', 'DISAMB'};
for ii=1:N
    %define global variables
   SimulateTraj_SE3_With_Blending;
   filename = strcat('SE3_DATA/DATASE3_', num2str(ii), '.mat');
   save(filename, 'ng', 'nd', 'cm', 'num_modes', 'xg_T', 'xr_T_true', 'intent_type', 'random_goal_index');
   for jj=1:length(vars)
        for kk=1:length(types)
            save(filename, strcat(vars{jj}, types{kk}), '-append');
        end
   end
   clear all; clc; close all;
   N = 400; %number of simulations
   vars = {'alpha_', 'curr_goal_', 'optimal_modes_', 'pgs_', 'traj_', 'uh_', 'ur_', 'blend_vel_'};
   types = {'POT', 'ENT', 'FI', 'DISAMB'};
end