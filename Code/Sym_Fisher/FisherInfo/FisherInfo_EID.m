clear; close all;
% close all;
ng = 3; %number of goals;
p_xg = (1/ng)*ones(ng, 1);
% p_xg = rand(4,1);
% p_xg = p_xg/sum(p_xg);
% fprintf('The prior goal distribution is given by %f\n', p_xg);
xg0 = zeros(2, ng);

%INSTANTIATE VALUES
xg0 = [ -3, 0;
        0, 0;
        -3, -3;
        3, -3]';
    
xg0 = [randsample(-4:4, ng);randsample(-4:4, ng)] + rand(2,1) - rand(2,1); %goal positions
% xr0 = [3.5,0.01]';
xr0 = [randsample(-4:4, 1);randsample(-4:4, 1)] + rand(2,1) - rand(2,1); %robot positions

sig0 = 0.1; %standard deviation for confidence random variable. 
uh0 = [-1,0]'; %positive x direction;
% uh0 = rand(2,1);


%PLOT GOALS AND ROBOT and UH
figure;
scatter(xg0(1,:), xg0(2,:), 180, 'r', 'filled'); grid on; hold on;
scatter(xr0(1), xr0(2), 140, 'k', 'filled');
quiver(xr0(1), xr0(2), uh0(1), uh0(2), 0, 'LineWidth', 2, 'MaxHeadSize', 2); %current robot velocity arrow
for i=1:ng %vectors connecting robot and goals. 
    quiver(xr0(1), xr0(2), xg0(1,i) - xr0(1), xg0(2,i) - xr0(2), 'LineWidth', 1.5, 'LineStyle', '-.');
end
xrange = [-5,5]; %set axis limits
yrange = [-5,5];
line(xrange, [0,0], 'Color', 'k'); %draw x and y axes. 
line([0,0], yrange, 'Color', 'k');
axis([xrange, yrange]);
axis square;
%define symbolic variables. Everything in the 2d position world for the
%time being. 

xr = sym('xr', [2,1], 'real');
xg = sym('xg', [2,1], 'real');
uh = sym('uh', [2,1], 'real');
PHI_XR_UH = sym('PHI', [2,2], 'real'); 
sig = sym('sig', 'real');


% %SYMBOLIC DEFINITIONS OF NORMS AND COS_TH

normx = (xg-xr)/norm(xg-xr);
normuh = uh/norm(uh);
cos_th = dot(normx, normuh); %confidence measure (sensor reading) for a particular goal x_g

% upsilon = (4*cos_th)/(1 - cos_th^2 + 10^(-10)); % c_g = upsilon + N(0, sig^2). MEASUREMENT MODEL. 10^-5 IS ADDED TO AVOID DIVIDE BY ZERO ISSUES
upsilon = (1 + cos_th)/2; %Measurement model. This 'voltage' meter can read from 0 to 1. 
% upsilon = piecewise(upsilon_raw > 0.95, 0.95, )
%COMPUTE THE PROPER FISHER INFORMATION MATRIX. CHECK MILLER ET AL. IN
%WHICH THE UNKNOWN PARAMETER IS A THE 2D LOCATION. 
FIM = (1/sig^2)*jacobian(gradient(upsilon, xg), xg);
% disp(eval(subs(eval(subs(FIM, [xg, xr, uh], [xg0(:,1), xr0, uh0])), sig, sig0)));
% disp(eval(subs(eval(subs(FIM, [xg, xr, uh], [xg0(:,2), xr0, uh0])), sig, sig0)));
%DISPLAY
% disp(eval(subs(normuh, uh, uh0)));
% disp(eval(subs(normx, [xg, xr], [xg0(:,2), xr0])));
% disp(eval(subs(cos_th, [xg, xr, uh], [xg0(:,2), xr0, uh0])));
% disp(eval(subs(upsilon, [xg, xr, uh], [xg0(:,1), xr0, uh0])));
% disp(eval(subs(eval(subs(FIM, [xg, xr, uh], [xg0(:,2), xr0, uh0])), sig, sig0))); %TWO-STEP PROCESS SINCE SIG AND THE VECTORS DON'T MATCH. SHOULD BE AN EASIER WAY OUT. 

%MARGINALIZING OVER XG

PHI_XR_UH = sym(zeros(2,2)); %FIM 
for i=1:ng %perform integral over the goals. 
    PHI_XR_UH(1,1) = subs(subs(FIM(1,1), xg, xg0(:,i)), sig, sig0)*p_xg(i) + PHI_XR_UH(1,1);
    PHI_XR_UH(1,2) = subs(subs(FIM(1,2), xg, xg0(:,i)), sig, sig0)*p_xg(i) + PHI_XR_UH(1,2);
    PHI_XR_UH(2,1) = subs(subs(FIM(2,1), xg, xg0(:,i)), sig, sig0)*p_xg(i) + PHI_XR_UH(2,1);
    PHI_XR_UH(2,2) = subs(subs(FIM(2,2), xg, xg0(:,i)), sig, sig0)*p_xg(i) + PHI_XR_UH(2,2);
end

%MARGINALIZING OVER XR
PHI_UH = subs(PHI_XR_UH, xr, xr0); %ROBOT POSITION IS KNOWN FULLY. THEREFORE EXPECTATION OVER XR REDUCES TO A POINT ESTIMATE
%SUBSTITUTING THE VALUE OF UH

uhs = [1,0; %+x
       -1,0; %-x
       0,1; %+y
       0,-1 %-y
       ]';
eid_uhs = zeros(size(uhs,2), 1);
for i=1:size(uhs, 2)
    eid_uhs(i) = det(eval(subs(PHI_UH, uh, uhs(:,i))));
end
% disp(eid_uhs);
% PHI = subs(PHI_UH, uh, uh0);
%EXPECTED INFORMATION DENSITY FOR UH
% EID_UH = det(eval(PHI)); %D-optimality


fprintf('The EID for x direction is %f\n', sum(eid_uhs(1:2)));
fprintf('The EID for y direction is %f\n', sum(eid_uhs(3:4)));

% fprintf('The EID for uh is %f\n', EID_UH);

% [x_xr, y_xr] = meshgrid(-5:1:5, -5:1:5);
% x = x_xr(:);
% y = y_xr(:);
% xrs = [x,y];
% % PHI_UH = subs(PHI_XR_UH, xr, {x_xr, y_xr}');
% % eids_x = det(eval(subs(PHI_UH, uh, uhs(:,1)))) + det(eval(subs(PHI_UH, uh, uhs(:,2))));
% % eids_y = det(eval(subs(PHI_UH, uh, uhs(:,3)))) + det(eval(subs(PHI_UH, uh, uhs(:,4))));
% eids_x = zeros(length(x), 1);
% eids_y = zeros(length(y), 1);
% 
% for i=1:length(x)
%     PHI_UH = subs(PHI_XR_UH, xr, xrs(i,:)');
%     eids_x(i) = det(eval(subs(PHI_UH, uh, uhs(:,1)))) + det(eval(subs(PHI_UH, uh, uhs(:,2))));
%     eids_y(i) = det(eval(subs(PHI_UH, uh, uhs(:,3)))) + det(eval(subs(PHI_UH, uh, uhs(:,4))));
% end
% 
% figure;
% scatter3(xg0(1,:), xg0(2,:), zeros(ng,1), 180, 'r', 'filled'); grid on; hold on;
% scatter3(xr0(1), xr0(2), 0, 180, 'k', 'filled');
% surf(x_xr, y_xr, reshape(eids_x, 11, 11))
% axis([-5 5 -5,5 -20 20]);
% xlabel('X Direction'); ylabel('Y Direction'); zlabel('EID_X');
% quiver3(0,0, 20, uhs(1,1), uhs(2,1), 0,'LineWidth', 2, 'MaxHeadSize', 2);
% 
% figure;
% scatter3(xg0(1,:), xg0(2,:), zeros(ng,1), 180, 'r', 'filled'); grid on; hold on;
% scatter3(xr0(1), xr0(2), 0, 180, 'k', 'filled');
% surf(x_xr, y_xr, reshape(eids_y, 11, 11))
% axis([-5 5 -5,5 -20 20]);
% xlabel('X Direction'); ylabel('Y Direction'); zlabel('EID_Y');
% quiver3(0,0, 20, uhs(1,3), uhs(2,3), 0,'LineWidth', 2, 'MaxHeadSize', 2);



