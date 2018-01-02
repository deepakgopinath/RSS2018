clc; clear; close all; 

x = 0:0.01:1;
% a = 14.12; b = 7.76;
% y = betapdf(x, a, b);
% 
% m = (a - 1)/(a + b -2);
% var = (a*b)/(((a + b)^2)*(a + b + 1));
% fprintf('The mode and variance are %f and %f\n', m, var);
% plot(x, y, 'r'); grid on; hold on;

m = 0.98;
var = 0.001;

syms a b m var;

eq1 = (a - 1)/(a + b -2) == m;
eq2 = (a*b)/(((a + b)^2)*(a + b + 1)) == var;

[sola, solb] = solve(eq1, eq2, [a,b]);
% fprintf('The shape params are %f and %f\n', sola, solb);
a_list = eval(vpa(sola)); %evaluate symbolic solution.
b_list = eval(vpa(solb));

a = a_list(a_list > 1);
b = b_list(b_list > 1);
fprintf('The shape params are %f and %f\n', a, b);
y = betapdf(x, real(a), real(b));
plot(x, y, 'b'); grid on;

