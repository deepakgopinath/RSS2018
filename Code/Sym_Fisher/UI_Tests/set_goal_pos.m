function set_goal_pos( source, events )
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here

global xg_t ng nd;
xg_t = [randsample(-4:4, ng);randsample(-4:4, ng); randsample(-4:4, ng)] + rand(nd,1) - rand(nd,1); %random goal positions. These will be treated as fixed parameters.
plot_xr_xg([0.4, 0.4, 0.4]);
disp('In resample goal')
end

