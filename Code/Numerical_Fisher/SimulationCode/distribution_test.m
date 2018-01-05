clear all; clc; close all;
%%
N = 10000;
rs_array = zeros(N, 1); ds_array = zeros(N, 1);
max_ng = 6;
for i=1:N
    rs_array(i) = randsample(max_ng-1, 1) + 1;
    ds_array(i) = datasample(2:max_ng, 1);
end
%%
histogram(rs_array); 
figure;
histogram(ds_array);