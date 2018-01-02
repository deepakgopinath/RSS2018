function set_rob_pos(source, event)
    global xr_t;
    fprintf('New Robot position is %f, %f, %f\n', xr_t(1), xr_t(2), xr_t(3));
%     plot_xr;
    best_mode = compute_disamb;
    colors = {[1,0,0], [0,1,0], [0,0,1], [1,1,0], [1,0,1], [0,1,1], [0.08,0.08,0.08]};
    plot_xr_xg(colors{best_mode});
end

