clear all; clc; close all;
global num_modes;
ng = 4; %number of goals. is a fixed parameter
nd = 3; %dimensionality of the problem.
num_modes = 2;
sig_t = 0.1; %std deviation for the entropy measurement.
xg_t = [randsample(-4:4, ng);rand(1,ng)*2 - 1; randsample(1:4, ng)] + rand(nd,1) - rand(nd,1); %random goal positions. These will be treated as fixed parameters.
% xg_t= [1, 0, 3;
%     -1,0, 3;
%     3, 3, 3;
%     3, -3, -3]';
% xr_t = [randsample(-4:4, 1);randsample(-4:4, 1); randsample(-4:4, 1) ] + rand(nd,1) - rand(nd,1); %random current robot position. FIM would be evaluated at this robot position
xr_t = [0,0,0]';
uh_t = [1,0, 0]'; % user control command

%%
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
%     disp(conf(cos_ths));
    cgs(i) = simplify((1 + cos_ths(i))/2);
    
end
cgs = heaviside(cgs - 0.5);