function best_mode = compute_disamb
%UNTITLED11 Summary of this function goes here
%   Detailed explanation goes here
    global PHI_UH_THETA  xr xr_t xgs xg_t uh nd ng;
    disp(xr_t);
    PHI_UH_THETA_L = PHI_UH_THETA;
    for i=1:ng
        PHI_UH_THETA_L = subs(PHI_UH_THETA_L, xgs(:,i) , xg_t(:, i));
    end

    %1D interface in the 3D problem.
%   PHI FOR U_1
    bias_term = 10^-12;
    c1 = subs(PHI_UH_THETA_L, uh, [1, bias_term, bias_term]');
    c2 = subs(PHI_UH_THETA_L, uh, [-1, -bias_term, -bias_term]');
    PHI_U1 = 0.5*c1 + 0.5*c2;

    %PHI FOR U_2
    c1 = subs(PHI_UH_THETA_L, uh, [bias_term, 1, bias_term]');
    c2 = subs(PHI_UH_THETA_L, uh, [-bias_term,-1, -bias_term]');
    PHI_U2 = 0.5*c1 + 0.5*c2;

    %PHI FOR U_3
    c1 = subs(PHI_UH_THETA_L, uh, [bias_term, bias_term, 1]');
    c2 = subs(PHI_UH_THETA_L, uh, [-bias_term, -bias_term, -1]');
    PHI_U3 = 0.5*c1 + 0.5*c2;   

    PHI_U1_FINAL = subs(PHI_U1, xr, xr_t);
    PHI_U2_FINAL = subs(PHI_U2, xr, xr_t);
    PHI_U3_FINAL = subs(PHI_U3, xr, xr_t);

    EID_U1 = det(eval(PHI_U1_FINAL));
    EID_U2 = det(eval(PHI_U2_FINAL));
    EID_U3 = det(eval(PHI_U3_FINAL));
    EID_US = [EID_U1; EID_U2; EID_U3];
    
    fprintf('The Disambiguation metric values for the control modes \n')
    disp(EID_US);
    thresh = 10^-3;
    diff_p = squareform(pdist(EID_US)/max(abs(EID_US)));
    inds = find(diff_p < thresh);
    % eq_array = zeros(nd, 1); %% 12 or 21, 13 or 31, 23 or 32
    if length(inds) > 3
        [i,j] = ind2sub([nd, nd], inds);
        eq_ind = i== j;
        i(eq_ind) = []; j(eq_ind) = [];
        eq_modes = unique(i+j);
        if length(eq_modes) > 1
            fprintf('Control modes 1, 2 and 3 disambiguate equally\n');
            best_mode = 7; %the index in the color array
        else
            [ss, ii] = sort(EID_US, 'descend');
            diff_bw_max = pdist(ss(1:2))/max(abs(ss(1:2))); %only concerned about the difference between max and second max
            if diff_bw_max < thresh
                fprintf('Control modes %d and %d are the best and disambiguate equally\n', ii(1), ii(2));
                best_mode = ii(1) + ii(2) + 1;
            else
                fprintf('Control mode %d disambiguate the best\n', ii(1));
                best_mode = ii(1);
            end
%             fprintf('Control modes %d and %d are equivalent\n', j(1), j(2));
%             fprintf('Control mode %s disambiguates the best \n', num2str(find(EID_US == max(EID_US))));
        end
    else
        fprintf('Control mode %s disambiguates the best \n', num2str(find(EID_US == max(EID_US))));
        best_mode = find(EID_US == max(EID_US));
    end
%     disp('In disamb compute function');
end

