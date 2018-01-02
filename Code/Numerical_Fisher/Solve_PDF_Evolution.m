clear all; close all;
clc; figure;

tspan = 15;
dt = 0.05;
timestamps = 0:dt:tspan;
ng = 4;
h = 1/ng;
tau = 1.2;
c =  3.0;
y0 = (1/ng)*ones(ng, 1);
yvals = zeros(length(timestamps) - 1,4);
ent_yvals = zeros(length(timestamps) - 1,1);
for i=1:length(timestamps)-1
    [t,y] = ode45(@(t,y) dft(t,y,h,tau,c, ng),[timestamps(i), timestamps(i+1)] , y0);
    yvals(i,:) = y(end, :);
    y0 = y(end,:);
    y0(y0 <= 0) = realmin; %keep it all positive. 
    y0 = y0/sum(y0);% normalize sum to 1 for a valid pdf. 
    ent_yvals(i) = sum(y0.*log2(y0)); %entropy vals. 
end

plot(timestamps(1:end-1),yvals(:,1), '-o', 'Color','r'); hold on;
plot(timestamps(1:end-1), yvals(:,2), '-o', 'Color', 'g');
plot(timestamps(1:end-1), yvals(:,3), '-o', 'Color', 'b');
plot(timestamps(1:end-1), yvals(:,4), '-o', 'Color', 'k');
% plot(t,y(:,1),'-o',t,y(:,2),'-o')
% axis([0, tspan,-1, max(ent_yvals)]); 
grid on; xlabel('Time'); ylabel('Activation');
line([0,tspan], [h, h], 'Color', 'r', 'LineWidth', 1);

hold on;
% plot(timestamps(1:end-1), ent_yvals, 'LineWidth', 1.5);