clear; clc; close all;

%Jay Myung MLE Tutorial. Likelihood Functions

w = 0:0.01:1;

w = w + (w == 0)*10^-5 - (w == 1)*10^-5;

%for y = 7, n=10
L = (factorial(10)/(factorial(7)*factorial(3))).*(w.^7).*((1 - w).^3);
logL = log((factorial(10)/(factorial(7)*factorial(3)))) + 7*log(w) + 3*log(1 - w);
% plot(w, L); grid on; 
% figure;
% plot(w, logL); grid on;

syms w;

L = (factorial(10)/(factorial(7)*factorial(3)))*(w^7)*((1 - w)^3);
logL = log(L);
score = diff(logL, w);
