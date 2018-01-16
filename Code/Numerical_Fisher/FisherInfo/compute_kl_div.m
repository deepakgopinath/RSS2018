function [ kl_div ] = compute_kl_div( pg_projected, pg_baseline ) %p i
%COMPUTE_KL_DIV Summary of this function goes here
%   Detailed explanation goes here
p = pg_projected; q = pg_baseline;
kl_div = -nansum(p.*log2(q./p));
end

