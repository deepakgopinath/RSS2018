clear all; clc; close all;

%Test numerical integration. of f(x) between x_min and x_max. 

xmin = 0;
xmax = 2;
auc = 0; %area under curve. 

dx = 0.001;
sp = xmin:dx:xmax-dx;
for i=1:length(sp)
    auc = auc + sp(i)^2*dx;
end
disp(auc);