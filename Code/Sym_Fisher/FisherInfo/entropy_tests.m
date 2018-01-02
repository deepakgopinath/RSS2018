clear all; clc; close all;

a = (1/10)*ones(10,1);
b = [0.8, 1/9*ones(1,9)]; b = b/sum(b);

%I want ent_b to be HIGHER than ENT_B; This is possible with the following
%formula which is the NEGATIVE of SHANNON entropy. 
ent_a = -sum(a.*log2(a));
ent_b = -(sum(b.*log2(b)));
