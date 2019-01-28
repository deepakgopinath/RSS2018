clear all; clc; close all;

%%
tic
% parpool(4);
s = 0;
% parpool(4);
parfor (i=1:1000000, 8);
    s = s + log(i);
    if rand < 0.9
        s = s - log(i);
    end
end
toc