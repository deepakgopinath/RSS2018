%% 
clear; close all;
%%

%%
ng = 2; %number of goals. is a fixed parameter
nd = 2;
sig_t = 0.1; %std deviation for the entropy measurement.
% xg_t = [randsample(-4:4, ng);randsample(-4:4, ng)] + rand(2,1) - rand(2,1); %random goal positions. These will be treated as fixed parameters.
xg_t= [
        -3, 3;
        3, 3]';
% xr_t = [randsample(-4:4, 1);randsample(-4:4, 1)] + rand(2,1) - rand(2,1); %random current robot position. FIM would be evaluated at this robot position
xr_t = [1, 0]';
uh_t = [1,0]'; % user control command
%%

%%
%PLOT GOALS AND ROBOT and UH
figure;
scatter(xg_t(1,:), xg_t(2,:), 180, 'r', 'filled'); grid on; hold on;
scatter(xr_t(1), xr_t(2), 140, 'k', 'filled');
quiver(xr_t(1), xr_t(2), uh_t(1), uh_t(2), 0, 'LineWidth', 2, 'MaxHeadSize', 2); %current robot velocity arrow
for i=1:ng %vectors connecting robot and goals.
    quiver(xr_t(1), xr_t(2), xg_t(1,i) - xr_t(1), xg_t(2,i) - xr_t(2), 'LineWidth', 1.5, 'LineStyle', '-.');
end
xrange = [-5,5]; %set axis limits
yrange = [-5,5];
line(xrange, [0,0], 'Color', 'k'); %draw x and y axes.
line([0,0], yrange, 'Color', 'k');
axis([xrange, yrange]);
axis square;
%%

%symbolic variables.
xr = sym('xr', [nd,1], 'real');
uh = sym('uh', [nd,1], 'real');
sig = sym('sig', 'real');
normuh = uh/norm(uh);
xgs = sym(zeros(nd, ng));
normxs = sym(zeros(nd, ng)); %unit vector connecting each goal in xgs and xr. There will be ng such unit vectors and each will have two components
cos_ths = sym(zeros(1, ng));
cgs = sym(zeros(1, ng));
pcgs = sym(zeros(1,ng));
for i=1:ng
    xgs(:, i) = sym(sprintf('xg%d', i), [nd,1], 'real'); %each goal location is a 2d vector xg1 = [xg11, xg12], xg2 = [xg21, xg22] and so on and so forth
    normxs(:, i) = (xgs(:,i)-xr)/norm(xgs(:,i)-xr);
    cos_ths(i) = dot(normxs(:,i), normuh);
    cgs(i) = (1 + cos_ths(i))/2;
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

%upsilon represents the ``virtual'' sensor reading.
upsilon = sum(pcgs.*log2(pcgs)); %negative of entropy of prob distribution
% upsilon = subs(upsilon, xr, xr_t);
% upsilon = subs(upsilon, sig, sig_t);
% upsilon = subs(upsilon, uh, uh_t); %substitute a control command
for i=1:ng
    upsilon = subs(upsilon, xgs(:, i), xg_t(:,i));
end
FIM = (1/sig^2)*jacobian(gradient(upsilon, xr), xr); %Fisher Information NMatrix. 2 by 2 because this is a 2D problem.
%
% cell_old = cell(1, ng+1);
% cell_old{1} = xr;
% for i=1:ng
%     cell_old{i+1} = xgs(:, i);
% end
% cell_new = cell(1, ng+1);
% cell_new{1} = xr_t;
% for i=1:ng
%     cell_new{i+1} = xg_t(:,i);
% end
PHI_UH = subs(FIM, sig, sig_t);
for i=1:ng
    PHI_UH = subs(PHI_UH, xgs(:,i) , xg_t(:, i));
end

% PHI_UH_VAL = subs(PHI_UH, xr, xr_t);
% uhs = [1,0; %+x
%            -1,0; %-x
%            0,1; %+y
%            0,-1 %-y
%            ]';
%
% eid_uhs = zeros(size(uhs,2), 1);
% for i=1:size(uhs, 2)
%     eid_uhs(i) = det(eval(subs(PHI_UH_VAL, uh, uhs(:,i))));
% end
% disp(eid_uhs);
% fprintf('The EID for x direction is %f\n', sum(eid_uhs(1:2)));
% fprintf('The EID for y direction is %f\n', sum(eid_uhs(3:4)));
%

%
% num_steps = 10;
% min_x = -2;
% max_x = 2;
% step_size = (max_x - min_x)/num_steps;
% x_vals = [ones(1,num_steps);min_x:step_size:max_x - step_size];
% eid_x = zeros(num_steps, 1);
% eid_y = zeros(num_steps, 1);
% uhs = [1,0; %+x
%            -1,0; %-x
%            0,1; %+y
%            0,-1 %-y
%            ]';
%
% eid_uhs = zeros(size(uhs,2), 1);
% for k=1:length(x_vals)
%     PHI_UH_VAL = subs(PHI_UH, xr, x_vals(:,k));
%
%     for i=1:size(uhs, 2)
%         eid_uhs(i) = det(eval(subs(PHI_UH_VAL, uh, uhs(:,i))));
%     end
%     fprintf('************************** x_val = %f %%%%%%%%%%%%%%%%%%\n', x_vals(:,k));
%     fprintf('The EID for x direction is %f\n', sum(eid_uhs(1:2)));
%     fprintf('The EID for y direction is %f\n', sum(eid_uhs(3:4)));
%     eid_x(k) = sum(eid_uhs(1:2));
%     eid_y(k) = sum(eid_uhs(3:4));
% end
% figure; hold on; plot(x_vals(2, :), eid_x); plot(x_vals(2,:), eid_y, 'r'); grid on;
