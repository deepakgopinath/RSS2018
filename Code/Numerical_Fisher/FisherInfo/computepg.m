function pg = computepg( uh, xr )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
global ng xg;
if size(xr, 2) > 1
    xr = xr';
end
pg = (1/ng)*zeros(ng, 1);
for i=1:ng
    normxs = (xg(:, i) - xr)/(norm((xg(:, i) - xr)) + realmin); %ith goal position;
    normuh = uh/(norm(uh) +realmin); %add realmin to avoid divide by zero errors. 
    costh = dot(normxs, normuh);
%     if costh < 0
%         pg(i) = 0;
%         pg(i) = (1 + dot(normxs, normuh))/2; %This is where ANY kind of method to generate P(g) can be used. 
% 
%     else
%         pg(i) = (1 + dot(normxs, normuh))/2; %This is where ANY kind of method to generate P(g) can be used. 
%     end
    pg(i) = (1 + dot(normxs, normuh))/2;
end
pg = pg/sum(pg);

end

