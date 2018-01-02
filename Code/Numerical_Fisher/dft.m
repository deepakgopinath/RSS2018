% function dydt = dft(t,y,h, tau, c)
% %UNTITLED3 Summary of this function goes here
% %   Detailed explanation goes here
% 
% dydt = (1.0/tau)*(-y + h + inputs(t) + c*sigmoid(y));
% end

function dydt = dft(t,y,h,tau,c,ng)
   dydt = zeros(ng,1);
   inp = inputs(t, ng); %2 d array. containing 
   lambda = -2*ones(ng,ng);
   lambda(1: ng+1: ng*ng) = 0;
   lambda = 4*eye(ng) + lambda;
   dydt = (-1/tau)*eye(ng)*y + (h/tau)*ones(ng,1) + lambda*sigmoid(inp);
end


function out = inputs(t, ng)
    if t < 1.0  || (t >= 6.0 && t < 10.0) || t >= 11.0 %control command has not been issued. 
        out = zeros(ng, 1);
    elseif t >= 1.0 && t < 3.0 %control command towards goal 1 and away from goal 2
        out = [0.9, 0.05, 0.1, 0.1]';
    elseif t >= 3.0 && t < 6.0
        out = [0.05, 0.9, 0.01, 0.1]';
%         out = zeros(ng, 1);
    elseif t >=10.0 && t < 11
         out = [0.1, 0.1, 0.9, 0.8]';
%         out = zeros(ng, 1);
    end
%     out = zeros(ng, 1);
%     fprintf('%f\t%f\n', out, t);
end

function out = sigmoid(u)
    out = 1./(1 + exp(-u));
    out = out - 0.5;
%     out = out';
%     fprintf('%f\n', out);
end
