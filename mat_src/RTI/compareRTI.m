function [ output_args ] = compare_quasiRTI( input_args )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
close all
load('logs/rti_compare_20.mat');

%% Analyzing mincut count
mc_avg = [];
mc_std = [];
data_log = [];

for exp_id=1:20
    idx = find(mincutCount(:,2) == exp_id);
    exp_mc = mincutCount(idx,1);
    mc_avg = [mc_avg; mean(exp_mc)];
    mc_std = [mc_std; std(exp_mc)];
    data_log = [data_log [exp_mc]];
end

figure
subplot(3,3,1)
bar(mc_avg)
xlabel('Patch no')
ylabel('Mincut Mean')
% figure
subplot(3,3,2)
bar(mc_std)
xlabel('Patch no')
ylabel('Mincut Standard Deviation')
% figure
subplot(3,3,3)
boxplot(data_log)
xlabel('Patch no')
ylabel('Mincut')

%% Analyzing RTI
rti_avg = [];
rti_std = [];
data_log = [];

for exp_id=1:50
    idx = find(mincutCount(:,3) == exp_id);
    exp_mc = mincutCount(idx,1);
    RTI = exp_mc/max(exp_mc);
    
    data_log = [data_log; [RTI]'];
end
rti_avg = mean(data_log,1);
rti_std = std(data_log,0,1);
    
% figure
subplot(3,3,4)
bar(rti_avg)
xlabel('Patch no')
ylabel('RTI Mean')
% figure
subplot(3,3,5)
bar(rti_std)
xlabel('Patch no')
ylabel('RTI Standard Deviation')
% figure
subplot(3,3,6)
boxplot(data_log)
xlabel('Patch no')
ylabel('RTI')

% hold on
% load('long_road_quasi2')
% maxval = max(mincutCount);
% mincutCount = [1; mincutCount/maxval];
% plot(mincutCount,'k*')

%% Analyzing RSI
rsi_avg = [];
rsi_std = [];
data_log = [];

for exp_id=1:50
    idx = find(mincutCount(:,3) == exp_id);
    exp_mc = safeMincutCount(idx,1);
    RSI = exp_mc/max(exp_mc);
    
    data_log = [data_log; [RSI]'];
end
rsi_avg = mean(data_log,1);
rsi_std = std(data_log,0,1);
    
% figure
subplot(3,3,7)
bar(rsi_avg)
xlabel('Patch no')
ylabel('RSI Mean')
% figure
subplot(3,3,8)
bar(rsi_std)
xlabel('Patch no')
ylabel('RSI Standard Deviation')
% figure
subplot(3,3,9)
boxplot(data_log)
xlabel('Patch no')
ylabel('RSI')

% road_space = [6;5.6;5.2;4.8;4.4;4;3.5;3;2.5;2];
% clearance = (road_space-1.58)/2;
% RSI = clearance/max(clearance);
% bar([RTI(1:10) RSI]);
% hold on
% load('long_roads_quasi_20')
% maxval = max(mincutCount);
% mincutCount = [mincutCount/maxval];
% plot(mincutCount,'k*')

figure
road_space = [6;5.6;5.2;4.8;4.4;4;3.5;3;2.5;2];
clearance = (road_space-1.58)/2;
RSI = clearance/max(clearance);

load('long_roads_quasi_20')
% maxval = max(safeMincutCount);
safeMincutCount = safeMincutCount(:,1);
safeMincutCount = [safeMincutCount/safeMincutCount(1)];

bar([safeMincutCount(1:10) RSI]);
end

