function [ output_args ] = compareMincut( input_args )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

close all
adjDirectory = '/home/khan/phd_ws/traversability/adjacency/long_roads_quasi/tmp/road_long';
configDirectory =  '/home/khan/phd_ws/traversability/configs/long_roads_quasi/tmp/road_long';

file_id = 1;

x_min = -3;
x_max = 3;
y_min = -10;
y_max = 10;
x_config_count = 25;
y_config_count = 81;
theta_config_count = 7;
costLog = [];
costMatLog = [];

    adjFile = strcat(adjDirectory,int2str(file_id),'_adj');
    configFile = strcat(configDirectory,int2str(file_id),'_conf');

Ds = importdata(adjFile,' ');
configs = importdata(configFile,' ');

startNodes = find(configs(:,2) < y_min+0.5);
goalNodes = find(configs(:,2) > y_max-0.5);
total_config = size(configs,1);

Ds(:,1:2) = Ds(:,1:2) +1;
adj_mat = Ds;
% Ds(:,3) = configs(Ds(:,1),3);
Ds = [Ds; [Ds(:,2) Ds(:,1) Ds(:,3)]];
Ds = [Ds;[total_config total_config 0]];
adj_mat = [adj_mat;[total_config total_config 0]];

% convert into sparse matrix and then graph
As = spconvert(Ds);
A = full(As);
degree = sum(A,2);
% adj_mat(:,3) = degree(adj_mat(:,1));
G = digraph(spconvert(adj_mat));

xCord = configs(:,1);
yCord = configs(:,2);
h = plot(G,'XData',xCord,'yData',yCord,'MarkerSize',3);
hold on

valid_idx = find(configs(:,5)==1);
valid = configs(valid_idx,:);
invalid_idx = find(configs(:,5)==0);
invalid = configs(invalid_idx,:);

plot(valid(:,1),valid(:,2),'g.');

%% plot mincut
load('logs/long_roads_quasi_tmp.mat');
mc1 = minCutLog(find(minCutLog(:,5)==1),:);
plot(mc1(:,2),mc1(:,3),'s','MarkerSize',7,'MarkerFaceColor','b');

mc2 = minCutLog(find(minCutLog(:,5)==9),:);
plot(mc2(:,2),mc2(:,3),'s','MarkerSize',7,'MarkerFaceColor','r');

end

