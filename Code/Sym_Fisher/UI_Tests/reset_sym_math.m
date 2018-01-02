function reset_sym_math
%UNTITLED12 Summary of this function goes here
%   Detailed explanation goes here

global xgs xr uh ng nd PHI_UH_THETA sig;
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
sig_t = 0.1;
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
end

