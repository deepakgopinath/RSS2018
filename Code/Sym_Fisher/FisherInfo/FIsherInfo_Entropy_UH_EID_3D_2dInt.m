%% 
clear all; clc; close all;
global num_modes;
ng = 4; %number of goals. is a fixed parameter
nd = 3; %dimensionality of the problem.
num_modes = 2;
sig_t = 0.1; %std deviation for the entropy measurement.
% xg_t = [randsample(-4:4, ng);rand(1,ng)*4 - 2; randsample(1:4, ng)] + rand(nd,1) - rand(nd,1); %random goal positions. These will be treated as fixed parameters.
xg_t= [3, -3, 3;
    3,3, -3;
    3, 3, 3;
    3, -3, -3]';
% xr_t = [randsample(-4:4, 1);randsample(-4:4, 1); randsample(-4:4, 1) ] + rand(nd,1) - rand(nd,1); %random current robot position. FIM would be evaluated at this robot position
xr_t = [0,0,0]';
uh_t = [1,0, 0]'; % user control command
%%

%PLOT GOALS AND ROBOT and UH
figure;
scatter3(xg_t(1,1:ng), xg_t(2,1:ng), xg_t(3,1:ng), 180, 'k', 'filled'); grid on; hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
scatter3(xr_t(1), xr_t(2), xr_t(3), 140, 'r', 'filled');
quiver3(xr_t(1), xr_t(2), xr_t(3), uh_t(1), uh_t(2), uh_t(3), 0, 'LineWidth', 2, 'MaxHeadSize', 2); %current robot velocity arrow
for i=1:ng %vectors connecting robot and goals.
    quiver3(xr_t(1), xr_t(2), xr_t(3), xg_t(1,i) - xr_t(1), xg_t(2,i) - xr_t(2), xg_t(3,i) - xr_t(3), 'LineWidth', 1.5, 'LineStyle', '-.');
end
xrange = [-5,5]; %set axis limits
yrange = [-5,5];
zrange = [-5,5];
line(xrange, [0,0], [0,0], 'Color', 'r', 'LineWidth', 3.5); %draw x and y axes.
line([0,0], yrange, [0,0], 'Color', 'g','LineWidth', 3.5);
line([0,0], [0,0], zrange, 'Color', 'b','LineWidth', 3.5);
axis([xrange, yrange, zrange]);
axis square;
view([32,16]);

%%
%symbolic variables. 
xr = sym('xr', [nd,1], 'real');
uh = sym('uh', [nd,1], 'real');
sig = sym('sig', 'real');
% normuh = uh/norm(uh);
normuh = uh;
xgs = sym(zeros(nd, ng));
normxs = sym(zeros(nd, ng)); %unit vector connecting each goal in xgs and xr. There will be ng such unit vectors and each will have two components
cos_ths = sym(zeros(1, ng));
cgs = sym(zeros(1, ng));
pcgs = sym(zeros(1,ng));
for i=1:ng
    xgs(:, i) = sym(sprintf('xg%d', i), [nd,1], 'real'); %each goal location is a 2d vector xg1 = [xg11, xg12], xg2 = [xg21, xg22] and so on and so forth
    normxs(:, i) = simplify((xgs(:,i)-xr)/norm(xgs(:,i)-xr));
    cos_ths(i) = simplify(dot(normxs(:,i), normuh));
    cgs(i) = simplify((1 + cos_ths(i))/2);
%     cgs(i) = piecewise(cgs(i) < 0.5, 0, cgs(i));
end
% cgs = subs(cgs, xr, xr_t);
% cgs = subs(cgs, uh, uh_t);
% for i=1:ng
%     cgs = subs(cgs, xgs(:, i), xg_t(:, i));
% end

%Confidences as probabilities. 
for i=1:ng
    pcgs(i) = cgs(i)/sum(cgs);
end
upsilon = simplify(sum(pcgs.*log2(pcgs))); %negative of entropy of prob distribution
% PHI_UH_THETA = upsilon;
PHI_UH_THETA = (1/sig^2)*jacobian(gradient(upsilon, uh), uh); %Fisher Information NMatrix. 2 by 2 because this is a 2D problem. 

% upsilon = subs(upsilon, xr, xr_t);
% upsilon = subs(upsilon, sig, sig_t);
% upsilon = subs(upsilon, uh, uh_t); %substitute a control command
% for i=1:ng
%     upsilon = subs(upsilon, xgs(:, i), xg_t(:,i));
% end
%%
% cgs = subs(cgs, xr, xr_t);
% cgs = subs(cgs, uh, [-1,0,0]');
% for i=1:ng
%     cgs = subs(cgs, xgs(:, i), xg_t(:, i));
% end
% pcgs = eval(cgs./sum(cgs));
% e = sum(pcgs.*log2(pcgs+realmin));
% disp(eval(cgs));
% disp(pcgs);
% disp(e);
%%

%substitute theta parameters
PHI_UH_THETA = subs(PHI_UH_THETA, sig, sig_t);
for i=1:ng
    PHI_UH_THETA = subs(PHI_UH_THETA, xgs(:,i) , xg_t(:, i));
end

%%
%1D interface in the 3D problem.
%PHI FOR U_1
uh_U1 = [1, 0, 0; -1, 0, 0]';
bias_term = 10^-2;
c1 = subs(PHI_UH_THETA, uh, [1, bias_term, bias_term]');
c2 = subs(PHI_UH_THETA, uh, [-1, -bias_term -bias_term]');
PHI_U1 = 0.5*c1 + 0.5*c2;

%PHI FOR U_2
uh_U2 = [0, 1, 0; 0, -1, 0]';
c1 = subs(PHI_UH_THETA, uh, [bias_term, 1, bias_term]');
c2 = subs(PHI_UH_THETA, uh, [-bias_term,-1, -bias_term]');
PHI_U2 = 0.5*c1 + 0.5*c2;

%PHI FOR U_3
uh_U3 = [0, 0, 1; 0, 0, -1]';
c1 = subs(PHI_UH_THETA, uh, [bias_term, bias_term, 1]');
c2 = subs(PHI_UH_THETA, uh, [-bias_term, -bias_term, -1]');
PHI_U3 = 0.5*c1 + 0.5*c2;


%%
PHI_U1_FINAL = subs(PHI_U1, xr, xr_t);
PHI_U2_FINAL = subs(PHI_U2, xr, xr_t);
PHI_U3_FINAL = subs(PHI_U3, xr, xr_t);
%%
EID_U1 = det(eval(PHI_U1_FINAL));
EID_U2 = det(eval(PHI_U2_FINAL));
EID_U3 = det(eval(PHI_U3_FINAL));
EID_US = [EID_U1; EID_U2; EID_U3];
disp(EID_US);


%%
% % Figure out how to detect close by values.     
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
    else
        [ss, ii] = sort(EID_US, 'descend');
        diff_bw_max = pdist(ss(1:2))/max(abs(ss(1:2)));
        if diff_bw_max < thresh
            fprintf('Control modes %d and %d are the best and disambiguate equally\n', ii(1), ii(2));
        else
            fprintf('Control mode %d disambiguate the best\n', ii(1));
        end
%         fprintf('Control modes %d and %d are equivalent\n', j(1), j(2));
%         fprintf('Control mode %s disambiguates the best \n', num2str(find(EID_US == max(EID_US))));
    end
%     for k=1:2:l % This only establishes pairwise equivalence. How to establish nested equivalence that can span across all hierarchices? 
%         fprintf('Control mode %s and %s are equivalent\n', num2str(i(k)), num2str(j(k)));
%     end
else
    fprintf('Control mode %s disambiguates the best \n', num2str(find(EID_US == max(EID_US))));
end
% 

%%
%2D interface with two modes. xy and xz
%THe prior is uniform. p(uh) = 1 for uh

dtheta = 0.6;
bias_term = 10^-12;
ang = 0:dtheta:2*pi-dtheta;
% PHI_U1 = sym(zeros(nd,nd));
EID_U1 = 0;
for i=1:length(ang)
    uh1  = cos(ang(i));
    uh2 =  sin(ang(i));
    uh_t = [uh1, uh2, bias_term]; %xy mode
    EID_U1 = EID_U1 + det(eval(subs(subs(PHI_UH_THETA, uh, uh_t')*dtheta, xr, xr_t)));
%     PHI_U1 = PHI_U1 + subs(PHI_UH_THETA, uh, uh_t')*dtheta;
end
%%
% PHI_U2 = sym(zeros(nd,nd));
EID_U2 = 0;
for i=1:length(ang)
    uh1  = cos(ang(i));
    uh2 =  sin(ang(i));
    uh_t = [uh1, bias_term, uh2]; %xz mode
%     PHI_U2 = PHI_U2 + subs(PHI_UH_THETA, uh, uh_t')*dtheta;
    EID_U2 = EID_U2 + det(eval(subs(subs(PHI_UH_THETA, uh, uh_t')*dtheta, xr, xr_t)));

end
%%
% PHI_U1_FINAL = subs(PHI_U1, xr, xr_t);
% PHI_U2_FINAL = subs(PHI_U2, xr, xr_t);
% 
% EID_U1 = det(eval(PHI_U1_FINAL));
% EID_U2 = det(eval(PHI_U2_FINAL));
clc;
EID_US = [EID_U1; EID_U2];
compute_best_mode(EID_US);
% fprintf('The EIDs for xy and xz modes are %f and %f respectively\n', EID_U1, EID_U2);
% if EID_U1 > EID_U2
%     fprintf('Control mode xy disambiguates best\n');
% elseif EID_U2 > EID_U1
%     fprintf('Control mode xz disambiguates best\n');
% else
%     fprintf('Control mode xy and xz disambiguate equally\n');
% end
% disp(EID_US);

%%
%% 
% Show best disambiguating modes for the entire workspace. 
min_ws = -4;
max_ws = 4;
num_steps = 3;
step_size = (max_ws - min_ws)/num_steps;
ax_p = (min_ws:step_size:max_ws)' +0.001;
[X,Y,Z] = meshgrid(ax_p);
ws_points = [X(:) Y(:) Z(:)];
disamb_modes = zeros(size(ws_points, 1), 1);
%%
for ii=1:size(ws_points,1)
    
    xr_t = ws_points(ii, :)';
    
%     PHI_U1_FINAL = subs(PHI_U1, xr, xr_t);
%     PHI_U2_FINAL = subs(PHI_U2, xr, xr_t);
% %     PHI_U3_FINAL = subs(PHI_U3, xr, xr_t);
%     
%     EID_U1 = det(eval(PHI_U1_FINAL));
%     EID_U2 = det(eval(PHI_U2_FINAL));
% %     EID_U3 = det(eval(PHI_U3_FINAL));
%     EID_US = [EID_U1; EID_U2];
    EID_U1 = 0; EID_U2 = 0;
    for i=1:length(ang)
        uh1  = cos(ang(i));
        uh2 =  sin(ang(i));
        uh_t = [uh1, uh2, bias_term]; %xy mode
        EID_U1 = EID_U1 + det(eval(subs(subs(PHI_UH_THETA, uh, uh_t')*dtheta, xr, xr_t)));
        uh_t = [uh1, bias_term, uh2]; %xz mode
        EID_U2 = EID_U2 + det(eval(subs(subs(PHI_UH_THETA, uh, uh_t')*dtheta, xr, xr_t)));
    end
    EID_US = [EID_U1; EID_U2];
%     fprintf('Control mode %s disambiguates the best \n', num2str(find(EID_US == max(EID_US))));
    disamb_modes(ii) = compute_best_mode(EID_US);
end

%%
%Show the results of the entire workspace simulations
figure;
scatter3(ws_points(disamb_modes == 1, 1), ws_points(disamb_modes == 1, 2),  ws_points(disamb_modes == 1, 3), 70, 'r', 'filled');grid on; hold on;
scatter3(ws_points(disamb_modes == 2, 1), ws_points(disamb_modes == 2, 2),  ws_points(disamb_modes == 2, 3), 70, 'g', 'filled');
scatter3(ws_points(disamb_modes == 3, 1), ws_points(disamb_modes == 3, 2),  ws_points(disamb_modes == 3, 3), 70, 'b', 'filled');
scatter3(xg_t(1,1:ng), xg_t(2,1:ng), xg_t(3,1:ng), 180, 'k', 'filled');
xrange = [-5,5]; %set axis limits
yrange = [-5,5];
zrange = [-5,5];
line(xrange, [0,0], [0,0], 'Color', 'k', 'LineWidth', 1.5); %draw x and y axes.
line([0,0], yrange, [0,0], 'Color', 'k','LineWidth', 1.5);
line([0,0], [0,0], zrange, 'Color', 'k','LineWidth', 1.5);
axis([xrange, yrange, zrange]);
axis square;xlabel('X'); ylabel('Y'); zlabel('Z');
view([32,16]);

function best_val = compute_best_mode(EID_US)
%     
%     fprintf('The Disambiguation metric values for the control modes \n')
%     disp(EID_US);
    global num_modes;
    thresh = 10^-3;
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