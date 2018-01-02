function [ Upsilon ] = compute_upsilon_pg( pg )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
Upsilon = nansum(pg.*log2(pg));
end

