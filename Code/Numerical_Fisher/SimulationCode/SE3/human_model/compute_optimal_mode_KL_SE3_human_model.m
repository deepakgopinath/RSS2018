function [ best_mode ] = compute_optimal_mode_KL_SE3_human_model( intent_type, xr_T, pg )
%COMPUTE_OPTIMAL_MODE_KL_R2_HUMAN_MODEL Summary of this function goes here
%   Detailed explanation goes here

global cm xg_T ng;

%for each mode in cm forward project using full human model and alpha value
%etc. 
EID_AR = zeros(length(cm), 1);
for i=1:length(cm)
    curr_mode = cm{i};
    for j=1:ng
        curr_goal = xg_T(:,:,j);
        EID_AR(i) = EID_AR(i) + (1/ng)*(compute_projected_kl_SE3_human_model(curr_mode, curr_goal, intent_type, xr_T, pg));
    end
end
best_mode = compute_best_mode(EID_AR);
end



