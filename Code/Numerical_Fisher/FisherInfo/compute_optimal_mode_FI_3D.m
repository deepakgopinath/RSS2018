function [ best_mode ] = compute_optimal_mode_FI_3D(intent_type, curr_x, pg, interface_type )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
% global delta_t;
global sig;
% compute_p_of_g, pg, xr,
if strcmp(interface_type, '1d')
        uh_U1 = [1, 0, 0; -1, 0, 0]';
        c1 = (1/sig^2)*(NumHessian(@compute_entropy, uh_U1(:,1), intent_type, curr_x, pg));
        c2 = (1/sig^2)*(NumHessian(@compute_entropy, uh_U1(:,2), intent_type, curr_x, pg));
        PHI_U1 = 0.5*c1 + 0.5*c2;

        uh_U2 = [0, 1, 0; 0, -1, 0]';
        c1 = (1/sig^2)*(NumHessian(@compute_entropy, uh_U2(:,1), intent_type, curr_x, pg));
        c2 = (1/sig^2)*(NumHessian(@compute_entropy, uh_U2(:,2), intent_type, curr_x, pg));
        PHI_U2 = 0.5*c1 + 0.5*c2;

        uh_U3 = [0, 0, 1; 0, 0, -1]';
        c1 = (1/sig^2)*(NumHessian(@compute_entropy, uh_U3(:,1), intent_type, curr_x, pg));
        c2 = (1/sig^2)*(NumHessian(@compute_entropy, uh_U3(:,2), intent_type, curr_x, pg));
        PHI_U3 = 0.5*c1 + 0.5*c2;
        
        EID_U1 = det(PHI_U1);
        EID_U2 = det(PHI_U2);
        EID_U3 = det(PHI_U3);
        EID_US = [EID_U1; EID_U2; EID_U3];
%         disp(EID_US);
        best_mode = compute_best_mode_1d(EID_US);
elseif strcmp(interface_type, '2d')
    dtheta = 0.4;
    ang = 0:dtheta:2*pi-dtheta;
    EID_U1 = 0; EID_U2 = 0;
    for i=1:length(ang)
        uh1  = cos(ang(i));
        uh2 =  sin(ang(i));
        uh_t = [uh1, uh2, 0]'; %xy mode
        EID_U1 = EID_U1 + det((1/sig^2)*(NumHessian(@compute_entropy, uh_t, intent_type, curr_x, pg))*dtheta);
        uh_t = [uh1, 0, uh2]'; %xz mode
        EID_U2 = EID_U2 + det((1/sig^2)*(NumHessian(@compute_entropy, uh_t, intent_type, curr_x, pg))*dtheta);
    end
    EID_US = [EID_U1; EID_U2];
%     disp(EID_US);
    best_mode = compute_best_mode_2d(EID_US);
end

end

%%
function best_val = compute_best_mode_1d(EID_US)
    thresh = 10^-3;
    global num_modes;
    diff_p = squareform(pdist(EID_US)/max(abs(EID_US)));
    inds = find(diff_p < thresh);
    if length(inds) > num_modes
        [i,j] = ind2sub([num_modes, num_modes], inds);
        eq_ind = i== j;
        i(eq_ind) = []; j(eq_ind) = [];
        eq_modes = unique(i+j);
        if length(eq_modes) > 1
            fprintf('Control modes 1, 2 and 3 disambiguate equally\n');
            best_val = 7;
        else
            [ss, ii] = sort(EID_US, 'descend');
            diff_bw_max = pdist(ss(1:2))/max(abs(ss(1:2)));
            if diff_bw_max < thresh
                fprintf('Control modes %d and %d are the best and disambiguate equally\n', ii(1), ii(2));
                best_val = ii(1) + ii(2) + 1;
            else
                fprintf('Control mode %d disambiguate the best\n', ii(1));
                best_val = ii(1);
            end
        end
    else
        fprintf('Control mode %s disambiguates the best \n', num2str(find(EID_US == max(EID_US))));
        best_val = find(EID_US == max(EID_US));
    end
end
%%
function best_val = compute_best_mode_2d(EID_US)
%     
%     fprintf('The Disambiguation metric values for the control modes \n')
%     disp(EID_US);
    global num_modes;
    thresh = 10^-3;
    diff_p = squareform(pdist(EID_US)/max(abs(EID_US)));
    inds = find(diff_p < thresh);
    if length(inds) > num_modes
        fprintf('2D Control Mode 1 and 2 disambiguate equally!\n');
        best_val = 3;
    else
        fprintf('2D Control mode %s disambiguates the best \n', num2str(find(EID_US == max(EID_US))));
        best_val =  find(EID_US == max(EID_US));
    end
end

