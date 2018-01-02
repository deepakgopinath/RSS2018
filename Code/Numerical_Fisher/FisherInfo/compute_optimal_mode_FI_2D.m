function [ best_mode ] = compute_optimal_mode_FI_2D(intent_type, curr_x, pg, interface_type)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
% global delta_t;
global sig;
    % compute_p_of_g, pg, xr,
best_mode = 1;
if strcmp(interface_type, '1d')
    uh_U1 = [1, 0; -1, 0]';
    c1 = (1/sig^2)*(NumHessian(@compute_entropy, uh_U1(:,1), intent_type, curr_x, pg));
    c2 = (1/sig^2)*(NumHessian(@compute_entropy, uh_U1(:,2), intent_type, curr_x, pg));
    PHI_U1 = 0.5*c1 + 0.5*c2;

    uh_U2 = [0, 1; 0, -1]';
    c1 = (1/sig^2)*(NumHessian(@compute_entropy, uh_U2(:,1), intent_type, curr_x, pg));
    c2 = (1/sig^2)*(NumHessian(@compute_entropy, uh_U2(:,2), intent_type, curr_x, pg));
    PHI_U2 = 0.5*c1 + 0.5*c2;

    EID_U1 = det(PHI_U1);
    EID_U2 = det(PHI_U2);

    EID_US = [EID_U1; EID_U2];
%     disp(EID_US);
    best_mode  = compute_best_mode(EID_US);
else
end
end
%%
function best_val = compute_best_mode(EID_US)
    global num_modes;
    thresh = 10^-2;
    diff_p = squareform(pdist(EID_US)/max(abs(EID_US)));
    inds = find(diff_p < thresh);
    if length(inds) > num_modes
        fprintf('Control Mode 1 and 2 disambiguate equally!\n');
        best_val = 3;
    else
        fprintf('Control mode %s disambiguates the best \n', num2str(find(EID_US == max(EID_US))));
        best_val =  find(EID_US == max(EID_US));
    end

end

