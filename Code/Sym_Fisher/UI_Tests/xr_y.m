function xr_y(source, event)
    
    global xr_t;
    xr_t(2) = source.Value;
%     fprintf('The Y pos of robot is %f \n', xr_t(2));
    plot_xr_xg([0.4, 0.4, 0.4]);
end

