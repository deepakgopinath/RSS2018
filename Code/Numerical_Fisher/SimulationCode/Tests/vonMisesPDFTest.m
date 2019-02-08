clear all; close all; clc;

%%
N = 1000;
nd = 3;
xg = ones(nd, 1); %45 degtree angle
xr = zeros(nd,1); %robot is always at origin. 

mu = xg -xr;
kappa = 2; %concentration parameters
vonmises_samples = randvonMisesFisherm(nd, N, kappa, mu);
pdf = zeros(N, 1);
for i=1:N
    x = vonmises_samples(:, i);
    pdf(i) = vonMisesFisherPdf(x, mu, kappa);
end

figure;
scatter3(vonmises_samples(1, :), vonmises_samples(2, :), vonmises_samples(3,:), 'r', 'filled'); grid on; hold on;
xlabel('X'); ylabel('Y'); zlabel('Z'); axis([-2,2,-2,2,-2,2]); axis('square');

figure;
histogram(pdf);