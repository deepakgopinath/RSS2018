clc; clear; close all;
t = @(x) (x./(1-x) - (1-x)./x);

% c = (0,1) -> (-inf, inf) 

% x = 0:0.01:1;
% plot(x, t(x), 'r'); grid on; hold on;
% plot(x, x, 'b'); 

ng = 4;
x = sym('x', [3,1], 'real');
uh = sym('uh', [3,1], 'real');
xg = sym('xg', [3,1], 'real');

p_xg = (1/ng)*ones(ng, 1);
normx = (xg-x)/norm(xg-x);
normuh = uh/norm(uh);
costh = dot(normx, normuh);

upsilon = (4*costh)/(1 - costh^2 + 10^(-5));

x0 = [3,4,2]'; %current robot position
uh0 = [1, 0,0]'; %current user velocity
xg0 = [5,4,2]'; %current goal position

disp(eval(subs(normx, [x,xg], [x0,xg0])));
disp(eval(subs(normuh, uh, uh0)))
disp(eval(subs(costh, [x,xg,uh], [x0, xg0, uh0])));
disp(eval(subs(upsilon, [x,xg,uh], [x0,xg0,uh0])));

dups_dxg = gradient(upsilon, xg);
disp(eval(subs(dups_dxg, [x,xg,uh], [x0, xg0, uh0])));
% tc = simplify(x/(1-x) - (1-x)/x);

