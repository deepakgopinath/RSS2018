function plot_xgs
%UNTITLED10 Summary of this function goes here
%   Detailed explanation goes here

    global f xg_t ng;
    figure(f);
    scatter3(xg_t(1,1:ng), xg_t(2,1:ng), xg_t(3,1:ng), 220, 'k', 'filled'); grid on; hold on; 
end

