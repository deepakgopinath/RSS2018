clear all; clc; close all;

%%
N = 1000;
%% Create von Mises Fisher distribution on a circle along a direction

nd = 3;
xg = ones(nd, 1); %45 degtree angle
xr = zeros(nd,1); %robot is always at origin. 

mu = [-0.1226,
    0.0310,
   -0.0997];
kappa = 20; %concentration parameters
vonmises_samples = randvonMisesFisherm(nd, N, kappa, mu);

%%
% mean = 0;
% std = 0.5;
% gaussian_noise = normrnd(mean, std, nd, N);
% gaussian_noise_samples = repmat(mu, 1, N) + gaussian_noise;
% for i=1:N
%     gaussian_noise_samples(:, i) = gaussian_noise_samples(:, i)./norm(gaussian_noise_samples(:, i));
% end

%%
figure;
scatter3(vonmises_samples(1, :), vonmises_samples(2, :), vonmises_samples(3,:), 'r', 'filled'); grid on; hold on;
xlabel('X'); ylabel('Y'); zlabel('Z'); axis([-2,2,-2,2,-2,2]); axis('square');

% figure;
%  histogram2(vonmises_samples(1, :), vonmises_samples(2, :))
%  
 %%
% figure;
% scatter(gaussian_noise_samples(1, :), gaussian_noise_samples(2, :), 'b', 'filled'); grid on; hold on;
% xlabel('X'); ylabel('Y'); axis([-2,2,-2,2]); axis('square');
% figure;
% histogram2(gaussian_noise_samples(1, :), gaussian_noise_samples(2, :))