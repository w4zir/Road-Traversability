close all

load('logs/long_roads_quasi_20.mat');
% bins = 7;
bins = [45 60 75 90 105 120 135];

 idx = find(minCutLog(:,5) == 1);
 mc_configs = minCutLog(idx,1:4);
 
 hist1 = hist(mc_configs(:,4),bins);
 
 idx2 = find(minCutLog(:,5) == 4);
 mc_configs2 = minCutLog(idx2,1:4);
 
 hist2 = hist(mc_configs2(:,4),bins);
 
 idx3 = find(minCutLog(:,5) == 8);
 mc_configs3 = minCutLog(idx3,1:4);
 
 hist3 = hist(mc_configs3(:,4),bins);
 
 ratio1 = hist2./hist1;
 ratio2 = hist3./hist1;
 
 c1 = (6-1.58)/2;
%  c1 = (6-1.3)/2;
 c2 = (4.8-1.58)/2;
 c3 = (3-1.58)/2;
 
 r1 = c2/c1;
 r2 = c3/c1;
 
 a = 1