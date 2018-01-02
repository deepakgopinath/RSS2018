clear all; clc; close all;
%%
global num_modes ng nd xg xr sig;
ng = 4; %number of goals. is a fixed parameter
nd = 3; %dimensionality of the problem.
num_modes = 2;
sig = 0.1; %std deviation for the entropy measurement.
% xg = [randsample(-4:4, ng);rand(1,ng)*8 - 4; randsample(-4:4, ng)] + rand(nd,1) - rand(nd,1); %random goal positions. These will be treated as fixed parameters.
xg= [3, -3, 3;
    3, 3, 3;
    -3, 3, 3;
    3, -3, -3]';
% xr = [randsample(-4:4, 1);randsample(-4:4, 1); randsample(-4:4, 1) ] + rand(nd,1) - rand(nd,1); %random current robot position. FIM would be evaluated at this robot position
xr = [0,0,0]';
uh = [1,0, 0]'; % user control command
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
xrange = [-5,5]; %set axis limits
yrange = [-5,5];
zrange = [-5,5];
line(xrange, [0,0], [0,0], 'Color', 'k', 'LineWidth', 1.5); %draw x and y axes.
line([0,0], yrange, [0,0], 'Color', 'k','LineWidth', 1.5);
line([0,0], [0,0], zrange, 'Color', 'k','LineWidth', 1.5);
axis([xrange, yrange, zrange]);
axis square;
view([32,16]);
%%
%%
%2D interface with two modes. xy and xz
%THe prior is uniform. p(uh) = 1 for uh
dtheta = 0.4;
ang = 0:dtheta:2*pi-dtheta;
EID_U1 = zeros(length(ang), 1);
for i=1:length(ang)
    uh1  = cos(ang(i));
    uh2 =  sin(ang(i));
    uh_t = [uh1, uh2, 0]'; %xy mode
    EID_U1(i) = det((1/sig^2)*(NumHessian(@compute_entropy, uh_t))*dtheta);
end
%%
EID_U2 = zeros(length(ang), 1);
for i=1:length(ang)
    uh1  = cos(ang(i));
    uh2 =  sin(ang(i));
    uh_t = [uh1, 0, uh2]'; %xz mode
    EID_U2(i) = det((1/sig^2)*(NumHessian(@compute_entropy, uh_t))*dtheta);
end

%%
clc;
EID_US = [sum(EID_U1); sum(EID_U2)];
disp(EID_US);
compute_best_mode(EID_US);


%% 
% Show best disambiguating modes for the entire workspace. 
min_ws = -4;
max_ws = 4;
num_steps = 6;
step_size = (max_ws - min_ws)/num_steps;
ax_p = (min_ws:step_size:max_ws)' +0.001;
[X,Y,Z] = meshgrid(ax_p);
ws_points = [X(:) Y(:) Z(:)];
disamb_modes = zeros(size(ws_points, 1), 1);

%%
for ii=1:size(ws_points,1)
    
    xr = ws_points(ii, :)';
    EID_U1 = 0; EID_U2 = 0;
    for i=1:length(ang)
        uh1  = cos(ang(i));
        uh2 =  sin(ang(i));
        uh_t = [uh1, uh2, 0]'; %xy mode
        EID_U1 = EID_U1 + det((1/sig^2)*(NumHessian(@compute_entropy, uh_t))*dtheta);
        uh_t = [uh1, 0, uh2]'; %xz mode
        EID_U2 = EID_U2 + det((1/sig^2)*(NumHessian(@compute_entropy, uh_t))*dtheta);
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
scatter3(xg(1,1:ng), xg(2,1:ng), xg(3,1:ng), 380, 'k', 'filled');
xrange = [-5,5]; %set axis limits
yrange = [-5,5];
zrange = [-5,5];
line(xrange, [0,0], [0,0], 'Color', 'r', 'LineWidth', 3.5); %draw x and y axes.
line([0,0], yrange, [0,0], 'Color', 'g','LineWidth', 3.5);
line([0,0], [0,0], zrange, 'Color', 'b','LineWidth', 3.5);
axis([xrange, yrange, zrange]);
axis square;xlabel('X'); ylabel('Y'); zlabel('Z');
view([32,16]);

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