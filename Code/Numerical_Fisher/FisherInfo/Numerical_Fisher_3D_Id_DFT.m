% clear all; clc; close all;
% close all;
%%
global num_modes ng nd xg xr sig delta_t;
ng = 2;
nd = 3;
num_modes = 3;
% xg = [rand(1,ng)*4 - 2;rand(1,ng)* 4- 2; rand(1,ng)*8 - 4] + rand(nd,1) - rand(nd,1); %random goal positions. These will be treated as fixed parameters.
% xg = [rand(1,ng)*8 - 4; rand(1,ng)* 8 - 4 ; rand(1,ng)*8 - 4] + rand(nd,1) - rand(nd,1); %random goal positions. These will be treated as fixed parameters.
% xg = [2.345585086756016,  -0.654414913243983,   3.345585086756016;
%    0.068458668712525,  -0.931541331287475,   2.068458668712525;
%   -2.389203961866748,   1.610796038133252,  -3.389203961866748;];
% xg= [-3, 0, 0.01;
%     -1, 0.5, 0;
%     3, -0.2, 0.01;
%     1, 0.1, -0.01]';
% xg = circshift(xg, 1);
% xg = circshift(xg, 1);
% xg = [0.45, -0.438, 0.23;
%       -0.45, -0.330, 0.2;
%       0.1, -0.45, 0.3]';
% xg = [0.45, -0.438, 0.23;
%       -0.45, -0.330, 0.2;
%       0.1, -0.45, 0.3]';
% xr = [0.2, -0.1, 0.3]';
% xr = [0.405, -0.128, 0.287]';
xg= [3, -3, 3;
    3, 3, 3;
    3, 3, -3;
    3, -3, -3]';
% xr =[-0.083077724768694, %robot pos at traj time stamp 501; 
%   -0.649295673560288,
%    0.288730342677929]';

xr = [0,0,0];
uh = [-1,0,0]';
sig = 0.01;
delta_t = 0.1;
%%

%PLOT GOALS AND ROBOT and UH
figure;
scatter3(xg(1,1:ng), xg(2,1:ng), xg(3,1:ng), 180, 'k', 'filled'); grid on; hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
scatter3(xr(1), xr(2), xr(3), 140, 'r', 'filled');
quiver3(xr(1), xr(2), xr(3), uh(1), uh(2), uh(3), 0, 'LineWidth', 2, 'MaxHeadSize', 2); %current robot velocity arrow
for i=1:ng %vectors connecting robot and goals.
    quiver3(xr(1), xr(2), xr(3), xg(1,i) - xr(1), xg(2,i) - xr(2), xg(3,i) - xr(3), 'LineWidth', 1.5, 'LineStyle', '-.');
end
sf = 1;
xrange = sf*[-5,5]; %set axis limits
yrange = sf*[-5,5];
zrange = sf*[-5,5];
line(xrange, [0,0], [0,0], 'Color', 'k', 'LineWidth', 1.5); %draw x and y axes.
line([0,0], yrange, [0,0], 'Color', 'k','LineWidth', 1.5);
line([0,0], [0,0], zrange, 'Color', 'k','LineWidth', 1.5);
axis([xrange, yrange, zrange]);
axis square;
view([32,16]);
%%
%Initialize the probability distribution over goals. 
% pg =(1/ng)*ones(ng, 1);
%%
% H = NumHessian(@compute_entropy, uh);
% PHI_UH_THETA = (1/sig^2)*H;

%%
% uh_U1 = [1, 0, 0; -1, 0, 0]';
% c1 = (1/sig^2)*(NumHessian(@compute_entropy, uh_U1(:,1), 'conf', xr));
% c2 = (1/sig^2)*(NumHessian(@compute_entropy, uh_U1(:,2), 'conf', xr));
% PHI_U1 = 0.5*c1 + 0.5*c2;
% 
% uh_U2 = [0, 1, 0; 0, -1, 0]';
% c1 = (1/sig^2)*(NumHessian(@compute_entropy, uh_U2(:,1), 'conf', xr));
% c2 = (1/sig^2)*(NumHessian(@compute_entropy, uh_U2(:,2), 'conf', xr));
% PHI_U2 = 0.5*c1 + 0.5*c2;
% 
% uh_U3 = [0, 0, 1; 0, 0, -1]';
% c1 = (1/sig^2)*(NumHessian(@compute_entropy, uh_U3(:,1), 'conf', xr));
% c2 = (1/sig^2)*(NumHessian(@compute_entropy, uh_U3(:,2), 'conf', xr));
% PHI_U3 = 0.5*c1 + 0.5*c2;
%%
% EID_U1 = det(PHI_U1);
% EID_U2 = det(PHI_U2);
% EID_U3 = det(PHI_U3);
% EID_US = [EID_U1; EID_U2; EID_U3];
% disp(EID_US);
%%
% compute_best_mode(EID_US);

%% 
    pg = [0.05, 0.4, 0.05, 0.4];
    pg = (1/ng)*ones(ng, 1);
    curr_x = xr';
    intent_type = 'dft';
    
    %%
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
    disp(EID_US);
    fprintf('Using Fisher Information \n');
    best_mode = compute_best_mode_1d(EID_US);
        %%
    uh_U1 = [1, 0, 0; -1, 0, 0]';
    c1 = compute_entropy(uh_U1(:,1), intent_type, curr_x, pg);
    c2 = compute_entropy(uh_U1(:,2), intent_type, curr_x, pg);
    PHI_U1 = 0.5*c1 + 0.5*c2;

    uh_U2 = [0, 1, 0; 0, -1, 0]';
    c1 = compute_entropy(uh_U2(:,1), intent_type, curr_x, pg);
    c2 = compute_entropy(uh_U2(:,2), intent_type, curr_x, pg);
    PHI_U2 = 0.5*c1 + 0.5*c2;

    uh_U3 = [0, 0, 1; 0, 0, -1]';
    c1 = compute_entropy(uh_U3(:,1), intent_type, curr_x, pg);
    c2 = compute_entropy(uh_U3(:,2), intent_type, curr_x, pg);
    PHI_U3 = 0.5*c1 + 0.5*c2;
    
    %for scalars determinant doesnt do anything. but its ok. 
    EID_U1 = det(PHI_U1);
    EID_U2 = det(PHI_U2);
    EID_U3 = det(PHI_U3);
    EID_US = [EID_U1; EID_U2; EID_U3];
    disp(EID_US);
    fprintf('Using Entropy Information \n');
    best_mode = compute_best_mode_1d(EID_US);
%%
% evaluate for entire workspace. 

min_ws = -5;
max_ws = 5;
num_steps = 5;
step_size = (max_ws - min_ws)/num_steps;
ax_p = (min_ws:step_size:max_ws)' +0.001;
[X,Y,Z] = meshgrid(ax_p);
ws_points = [X(:) Y(:) Z(:)];
disamb_modes = zeros(size(ws_points, 1), 1);
pg0 = (1/ng)*ones(ng,1);
%%
for i=1:size(ws_points,1)
    xr = ws_points(i, :)';
    disamb_modes(i) = compute_optimal_mode_FI_3D('dft', xr, pg0, '1d');
end

%%
%%
%Show the results of the entire workspace simulations
figure;
scatter3(xg(1,1:ng), xg(2,1:ng), xg(3,1:ng), 580, 'k', 'filled');
% xrange = [-1,1]; %set axis limits
% yrange = [-1,1];
% zrange = [-1,1];
sf = 1.5;
xrange = sf*[-5,5]; %set axis limits
yrange = sf*[-5,5];
zrange = sf*[-5,5];
line(xrange, [0,0], [0,0], 'Color', 'r', 'LineWidth', 3.5); %draw x and y axes.
line([0,0], yrange, [0,0], 'Color', 'g','LineWidth', 5.5);
line([0,0], [0,0], zrange, 'Color', 'b','LineWidth', 3.5);
axis([xrange, yrange, zrange]);
axis square;xlabel('\bf X'); ylabel('\bf Y'); zlabel('\bf Z');
view([32,16]);
grid on; hold on;
scatter3(ws_points(disamb_modes == 1, 1), ws_points(disamb_modes == 1, 2),  ws_points(disamb_modes == 1, 3), 70, 'r', 'filled');
scatter3(ws_points(disamb_modes == 2, 1), ws_points(disamb_modes == 2, 2),  ws_points(disamb_modes == 2, 3), 70, 'g', 'filled');
scatter3(ws_points(disamb_modes == 3, 1), ws_points(disamb_modes == 3, 2),  ws_points(disamb_modes == 3, 3), 70, 'b', 'filled');
scatter3(ws_points(disamb_modes == 4, 1), ws_points(disamb_modes == 4, 2),  ws_points(disamb_modes == 4, 3), 70, 'y', 'filled');
scatter3(ws_points(disamb_modes == 5, 1), ws_points(disamb_modes == 5, 2),  ws_points(disamb_modes == 5, 3), 70, 'm', 'filled');
scatter3(ws_points(disamb_modes == 6, 1), ws_points(disamb_modes == 6, 2),  ws_points(disamb_modes == 6, 3), 70, 'c', 'filled');
scatter3(ws_points(disamb_modes == 7, 1), ws_points(disamb_modes == 7, 2),  ws_points(disamb_modes == 7, 3), 70, 'k', 'filled');

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

%%
% function [EID_US] = compute_FIM
%     global sig xr;
%     uh_U1 = [1, 0, 0; -1, 0, 0]';
%     c1 = (1/sig^2)*(NumHessian(@compute_entropy, uh_U1(:,1), 'conf', xr));
%     c2 = (1/sig^2)*(NumHessian(@compute_entropy, uh_U1(:,2), 'conf', xr));
%     PHI_U1 = 0.5*c1 + 0.5*c2;
% 
%     uh_U2 = [0, 1, 0; 0, -1, 0]';
%     c1 = (1/sig^2)*(NumHessian(@compute_entropy, uh_U2(:,1), 'conf', xr));
%     c2 = (1/sig^2)*(NumHessian(@compute_entropy, uh_U2(:,2), 'conf', xr));
%     PHI_U2 = 0.5*c1 + 0.5*c2;
% 
%     uh_U3 = [0, 0, 1; 0, 0, -1]';
%     c1 = (1/sig^2)*(NumHessian(@compute_entropy, uh_U3(:,1), 'conf', xr));
%     c2 = (1/sig^2)*(NumHessian(@compute_entropy, uh_U3(:,2), 'conf', xr));
%     PHI_U3 = 0.5*c1 + 0.5*c2;
%     EID_U1 = det(PHI_U1);
%     EID_U2 = det(PHI_U2);
%     EID_U3 = det(PHI_U3);
%     EID_US = [EID_U1; EID_U2; EID_U3];
%     disp(EID_US);   
% end
