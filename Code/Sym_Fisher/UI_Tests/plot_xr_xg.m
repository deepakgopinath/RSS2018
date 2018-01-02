function plot_xr_xg(xr_color)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
    global xr_t;
    delete(findobj(gcf, 'type', 'Scatter'))
    scatter3(xr_t(1), xr_t(2), xr_t(3), 100, xr_color, 'filled'); grid on; hold on;
    plot_xgs;
end

