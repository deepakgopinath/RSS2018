clear; clc; close all;

global xr_t f xgs xg_t ng nd uh uh_t PHI_U1 PHI_U2 PHI_U3 PHI_UH_THETA xr sig;

nd = 3;
ng = 3;
sig_t = 0.1; %std deviation for the entropy measurement.


xr_t = zeros(nd, 1);
uh_t = [1,0, 0]'; % user control command
% xg_t = [randsample(-4:4, ng);randsample(-4:4, ng); randsample(-4:4, ng)] + rand(nd,1) - rand(nd,1); %random goal positions. These will be treated as fixed parameters.
xg_t= [-3, -3, 3;
    3,-3, -3;
    3, 3, 3;
    -3, -3, 3]';
f = figure('Visible', 'off');
createSlidersAndButtons; %create the buttons and sliders for the interactive gui. 
f.Visible = 'on';

plot_init_xr; %plot the initial robot position
plot_xgs; %plot the initial goal positions

%%
%%SYMBOLIC MATH FOR FI METRIC CALCULATION
%symbolic variables and FI math.
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
    normxs(:, i) = (xgs(:,i)-xr)/norm(xgs(:,i)-xr);
    cos_ths(i) = dot(normxs(:,i), normuh);
    cgs(i) = (1 + cos_ths(i))/2;
end

%Confidences as probabilities. 
for i=1:ng
    pcgs(i) = cgs(i)/sum(cgs);
end
upsilon = sum(pcgs.*log2(pcgs)); %negative of entropy of prob distribution

PHI_UH_THETA = (1/sig^2)*jacobian(gradient(upsilon, uh), uh); %Fisher Information NMatrix. 2 by 2 because this is a 2D problem. 

%%

%substitute theta parameters
PHI_UH_THETA = subs(PHI_UH_THETA, sig, sig_t);
PHI_UH_THETA_L = PHI_UH_THETA;
for i=1:ng
    PHI_UH_THETA_L = subs(PHI_UH_THETA_L, xgs(:,i) , xg_t(:, i));
end

%1D interface in the 3D problem.
%PHI FOR U_1
uh_U1 = [1, 0, 0; -1, 0, 0]';
bias_term = 10^-12;
c1 = subs(PHI_UH_THETA_L, uh, [1, bias_term, bias_term]');
c2 = subs(PHI_UH_THETA_L, uh, [-1, -bias_term, -bias_term]');
PHI_U1 = 0.5*c1 + 0.5*c2;

%PHI FOR U_2
uh_U2 = [0, 1, 0; 0, -1, 0]';
c1 = subs(PHI_UH_THETA_L, uh, [bias_term, 1, bias_term]');
c2 = subs(PHI_UH_THETA_L, uh, [-bias_term,-1, -bias_term]');
PHI_U2 = 0.5*c1 + 0.5*c2;

%PHI FOR U_3
uh_U3 = [0, 0, 1; 0, 0, -1]';
c1 = subs(PHI_UH_THETA_L, uh, [bias_term, bias_term, 1]');
c2 = subs(PHI_UH_THETA_L, uh, [-bias_term, -bias_term, -1]');
PHI_U3 = 0.5*c1 + 0.5*c2;

%%


function plot_init_xr
    global xr_t;
    gcf;
    scatter3(xr_t(1), xr_t(2), xr_t(3), 100, [0.4, 0.4, 0.4], 'filled'); grid on; hold on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    xrange = [-5,5]; %set axis limits
    yrange = [-5,5];
    zrange = [-5,5];
    line(xrange, [0,0], [0,0], 'Color', 'r', 'LineWidth', 1.5); %draw x and y axes.
    line([0,0], yrange, [0,0], 'Color', 'g','LineWidth', 1.5);
    line([0,0], [0,0], zrange, 'Color', 'b','LineWidth', 1.5);
    axis([xrange, yrange, zrange]);
    axis square;
    view([32,16]);
end