clear all; clc; close all;

%%
global nd num_modes;
ng = 4; %number of goals. is a fixed parameter
num_modes = 2;
nd = 2; %dimensionality of the problem.
sig_t = 0.1; %std deviation for the entropy measurement.
xg_t = [randsample(-4:4, ng);randsample(-4:4, ng)] + rand(2,1) - rand(2,1); %random goal positions. These will be treated as fixed parameters.
% xg_t= [3,-3;
%     3,3;
%     3, -3;
%     -3, -3;]';
% xr_t = [randsample(-4:4, 1);randsample(-4:4, 1)] + rand(2,1) - rand(2,1); %random current robot position. FIM would be evaluated at this robot position
xr_t = [0,0]';
uh_t = [1,0]'; % user control command

%%
%PLOT GOALS AND ROBOT and UH
figure;
scatter(xg_t(1,1:ng), xg_t(2,1:ng), 180, 'k', 'filled'); grid on; hold on;
scatter(xr_t(1), xr_t(2), 140, 'r', 'filled');
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

%substitute theta parameters
PHI_UH_THETA = subs(PHI_UH_THETA, sig, sig_t);
for i=1:ng
    PHI_UH_THETA = subs(PHI_UH_THETA, xgs(:,i) , xg_t(:, i));
end
%PHI FOR U_1
uh_U1 = [1, 0; -1, 0]';
bias_term = 10^-16;
c1 = subs(PHI_UH_THETA, uh, [1, bias_term]');
c2 = subs(PHI_UH_THETA, uh, [-1, bias_term]');
PHI_U1 = 0.5*c1 + 0.5*c2;
%PHI FOR U_2
uh_U2 = [0, 1; 0, -1]';
c1 = subs(PHI_UH_THETA, uh, [bias_term, 1]');
c2 = subs(PHI_UH_THETA, uh, [-bias_term,-1]');
PHI_U2 = 0.5*c1 + 0.5*c2;
%%


PHI_U1_FINAL = subs(PHI_U1, xr, xr_t);
PHI_U2_FINAL = subs(PHI_U2, xr, xr_t);

EID_U1 = det(eval(PHI_U1_FINAL));
EID_U2 = det(eval(PHI_U2_FINAL));
EID_US = [EID_U1; EID_U2];
%%
disp(EID_US);
compute_best_mode(EID_US);
fprintf('The Disambiguation metric values for the control modes \n')
% disp(EID_US);
% thresh = 10^-3;
% diff_p = squareform(pdist(EID_US)/max(abs(EID_US)));
% inds = find(diff_p < thresh);
% if length(inds) > nd
%     fprintf('Control Mode 1 and 2 disambiguate equally!\n');
% else
%     fprintf('Control mode %s disambiguates the best \n', num2str(find(EID_US == max(EID_US))));
% end


%%
% if EID_U1 - EID_U2 > 10^-2
%     fprintf('Control Mode 1 disambiguates better\n');
% elseif EID_U1 - EID_U2 < -10^-2
%     fprintf('Control Mode 2 disambiguates better\n');
% else
%     fprintf('Control Mode 1 and 2 disambiguate equally!\n');
% end
% fprintf('EIDU1 - EIDU2 = %f\n', EID_U1 - EID_U2);
% %SAmple points in workspace uniformly. Plot pointcloud for this 2d case.
%%
min_ws = -4;
max_ws = 4;
num_steps = 9;
step_size = (max_ws - min_ws)/num_steps;
ax_p = (min_ws:step_size:max_ws)' +0.001;
[X,Y] = meshgrid(ax_p);
ws_points = [X(:) Y(:)];
disamb_modes = zeros(size(ws_points, 1), 1);
%%
for i=1:size(ws_points,1)

    xr_t = ws_points(i, :)';

    PHI_U1_FINAL = subs(PHI_U1, xr, xr_t);
    PHI_U2_FINAL = subs(PHI_U2, xr, xr_t);

    EID_U1 = det(eval(PHI_U1_FINAL));
    EID_U2 = det(eval(PHI_U2_FINAL));
    EID_US = [EID_U1; EID_U2];

    disamb_modes(i) = compute_best_mode(EID_US);
%     disp(i)
end

%
%%
figure;grid on; hold on;
colors = {[1,0,0], [0,1,0], [0,0,1], [1,1,0], [1,0,1], [0,1,1], [0.08,0.08,0.08]};

for i=1:length(colors)
    scatter(ws_points(disamb_modes == i, 1), ws_points(disamb_modes == i, 2), 70, colors{i}, 'filled'); hold on;
end
scatter(xg_t(1,1:ng), xg_t(2,1:ng), 180, 'k', 'filled');

xrange = [-5.5,5.5]; %set axis limits
yrange = [-5.5,5.5];
line(xrange, [0,0], 'Color', 'r'); %draw x and y axes.
line([0,0], yrange, 'Color', 'g');
axis([xrange, yrange]);
legend('Mode 1', 'Mode 2', 'Equivalent');
axis square;
xlabel('Spatial X'); ylabel('Spatial Y'); title('Best Disamb Control Modes');

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
