clear all; clc; close all; 

%%
% x = ainit(3,3);      % initialize audi variable, up to 3rd derivative covered
% y = func(x);

%%
p1 = 0.2; p2 = 0.5;
[x1, x2, x3] = ainit(1, 3, 0.5, 2);
x = [x1, x2, x3];
y = func(x, p1, p2);

h = ahess(y, 0);
