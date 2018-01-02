clear all; close all; clc;
%%

global num_modes ng nd xg xr sig delta_t;
ng = 3;
nd = 2; %2D world. 
num_modes = 2;

xg = [randsample(-4:4, ng);randsample(-4:4, ng)] + rand(2,1) - rand(2,1); %random goal positions. These will be treated as fixed parameters.
xr = [0,0];
uh = [1,0]'; % user control command
sig = 0.01;
delta_t = 0.1;

%%
%PLOT GOALS AND ROBOT and UH
figure;
scatter(xg(1,1:ng), xg(2,1:ng), 180, 'k', 'filled'); grid on; hold on;
scatter(xr(1), xr(2), 140, 'r', 'filled');
quiver(xr(1), xr(2), uh(1), uh(2), 0, 'LineWidth', 2, 'MaxHeadSize', 2); %current robot velocity arrow
for i=1:ng %vectors connecting robot and goals.
    quiver(xr(1), xr(2), xg(1,i) - xr(1), xg(2,i) - xr(2), 'LineWidth', 1.5, 'LineStyle', '-.');
end
xrange = [-5,5]; %set axis limits
yrange = [-5,5];
line(xrange, [0,0], 'Color', 'r'); %draw x and y axes.
line([0,0], yrange, 'Color', 'g');
axis([xrange, yrange]);
axis square;

%%
% pg = [0.05, 0.4, 0.05, 0.4];
pg = [0.4, 0.2, 0.4];
% pg = (1/ng)*ones(ng, 1);
curr_x = xr';
intent_type = 'dft';

%% FISHER BASED COMPUTATION
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
disp(EID_US);
compute_best_mode(EID_US);

%% ENTROPY BASED COMPUTATION.
uh_U1 = [1, 0; -1, 0]';
c1 = compute_entropy(uh_U1(:,1), intent_type, curr_x, pg);
c2 = compute_entropy(uh_U1(:,2), intent_type, curr_x, pg);
PHI_U1 = 0.5*c1 + 0.5*c2;

uh_U2 = [0, 1; 0, -1]';
c1 = compute_entropy(uh_U2(:,1), intent_type, curr_x, pg);
c2 = compute_entropy(uh_U2(:,2), intent_type, curr_x, pg);
PHI_U2 = 0.5*c1 + 0.5*c2;

EID_U1 = det(PHI_U1);
EID_U2 = det(PHI_U2);

EID_US = [EID_U1; EID_U2];
disp(EID_US);
compute_best_mode(EID_US);


%% simulate for entire workspace. 
min_ws = -5;
max_ws = 5;
num_steps = 12;
step_size = (max_ws - min_ws)/num_steps;
ax_p = (min_ws:step_size:max_ws)' +0.001;
[X,Y] = meshgrid(ax_p);
ws_points = [X(:) Y(:)];
disamb_modes = zeros(size(ws_points, 1), 1);
pg0 = (1/ng)*ones(ng,1);
% pg0 = [0.4, 0.2, 0.4];
%%
for i=1:size(ws_points,1)
    xr = ws_points(i, :)';
    disamb_modes(i) = compute_optimal_mode_ENT_2D('dft', xr, pg0, '1d');
end

%%
figure;grid on; hold on;
colors = {[1,0,0], [0,1,0], [0,0,1]};

for i=1:length(colors)
    scatter(ws_points(disamb_modes == i, 1), ws_points(disamb_modes == i, 2), 70, colors{i}, 'filled'); hold on;
end
scatter(xg(1,1:ng), xg(2,1:ng), 180, 'k', 'filled');

xrange = [-5.5,5.5]; %set axis limits
yrange = [-5.5,5.5];
line(xrange, [0,0], 'Color', 'r'); %draw x and y axes.
line([0,0], yrange, 'Color', 'g');
axis([xrange, yrange]);
legend('Mode 1', 'Mode 2', 'Equivalent');
axis square;
xlabel('Spatial X'); ylabel('Spatial Y'); title('Best Disamb Control Modes');

%%
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