function xr_x(source, event)
    
    global xr_t;
    xr_t(1) = source.Value;
%     fprintf('The X pos of robot is %f \n', xr_t(1));
    plot_xr_xg([0.4, 0.4, 0.4]);
end

