function [ Upsilon ] = compute_projected_entropy_SE3( uh, varargin )
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here

projection_time = 1;
projected_pg = compute_pg_projection_SE3(uh, projection_time, varargin{:});
% disp(projected_pg);
Upsilon = compute_pg_entropy(projected_pg);
% disp(Upsilon);

end

