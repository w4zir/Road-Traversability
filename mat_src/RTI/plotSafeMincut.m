function [ output_args ] = plotSafeMincut( input_args )
%Plot safemintcut computed using computeSafeMincut.m
%   Detailed explanation goes here


%% Initialization
clear ; close all; clc

configDirectory =  '/home/wazir/phd_ws/traversability/configs/rsi/rand_01/road_rsi';

frame_id = 3;


%% ==================== Part 1: load data ====================
load('safe_roads_rand.mat');

configFile = strcat(configDirectory,int2str(frame_id),'_conf');
configs = importdata(configFile,' ');

%% ==================== Part 2: plot data ====================
valid_idx = find(configs(:,5)==1);
valid = configs(valid_idx,:);
invalid_idx = find(configs(:,5)==0);
invalid = configs(invalid_idx,:);

idx = find(minCutLog(:,5) == frame_id);
minCutNodes = minCutLog(idx,:);

idx = find(safeMincutLog(:,5) == frame_id);
safeMinCutNodes = safeMincutLog(idx,:);

plot(valid(:,1),valid(:,2),'g.');
hold on
plot(invalid(:,1),invalid(:,2),'r.');

plot(minCutNodes(:,2),minCutNodes(:,3),'bo','MarkerSize',7,'MarkerFaceColor','r');
plot(safeMinCutNodes(:,2),safeMinCutNodes(:,3),'ko','MarkerSize',7,'MarkerFaceColor','g');

legend('Valid Configs', 'Invalid Configs', 'Safe Mincut Configs', 'Remaining Mincut Configs')
xlabel('Width')
ylabel('Length')
% plot(configs(startNodes,1),configs(startNodes,2),'gs');
% plot(configs(goalNodes,1),configs(goalNodes,2),'rs');
end

