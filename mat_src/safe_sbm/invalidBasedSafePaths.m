function [ output_args ] = invalidBasedSafePaths( input_args )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

close all
adj_directory = '/home/khan/phd_ws/traversability/adjacency/long_roads_quasi/road_long';
config_directory = '/home/khan/phd_ws/traversability/configs/long_roads_quasi/road_long';

file_id = 11;

min_x = -3;
max_x = 3;
min_y = -10;
max_y = 10;
min_t = 60;
max_t = 120;
x_config_count = 25;
y_config_count = 81;
theta_config_count = 7;
wheelbase = 1.78;
clearane_dist = 0.5;
high_cost = 100;

adjFile = strcat(adj_directory,int2str(file_id),'_adj');
configFile = strcat(config_directory,int2str(file_id),'_prt');

% Read prm and configuration files
prm_adj_list = importdata(adjFile,' ');
configs = importdata(configFile,' ');

% calculated prm distances
prm_adj_list(:,1:2) = prm_adj_list(:,1:2) +1;
prm_nodes_coord = configs(prm_adj_list(:,1),1:3);
prm_neighbors_coord = configs(prm_adj_list(:,2),1:3);
xy_diff = prm_nodes_coord(:,1:2) - prm_neighbors_coord(:,1:2);
theta_diff = abs(prm_nodes_coord(:,3) - prm_neighbors_coord(:,3))*pi./180;
% prm_distances = sqrt(sum((xy_diff).^2,2) + wheelbase^2 * (theta_diff).^2);
prm_distances = sqrt(sum((xy_diff).^2,2));

% find clearance of each node using neighbor invalid nodes
% 1. find valid and invalid node ids
startNodes = find(configs(:,2) < min_y+0.5 & configs(:,3) > 85 & configs(:,3) < 95);
goalNodes = find(configs(:,2) < 5.4 & configs(:,2) > 5);
total_configs = size(configs,1);

nodes = [1:total_configs]';
valid_nodes = find(configs(:,5)==1);
invalid_nodes = find(configs(:,5)==0);

% 2. call the function to compute clearance of each config
configs_clearance = invalidBasedConfigsClearance(configs,prm_adj_list,prm_distances,valid_nodes,invalid_nodes);
configs_clearance = 1./configs_clearance;
% prm_adj_list(:,3) = configs_clearance(prm_adj_list(:,2));
% configs_clearance = configs(:,6);

% get prm adjacency list along with their distances
prm_adj_without_clearance = [prm_adj_list(:,1:2) prm_distances];

% adjust prm adjacency list by adding clearance constraint
% configs_clearance = valid_prm_adj(:,3);
% prm_adj_with_clearance = addClearanceConstraint2prm(prm_adj_without_clearance,configs_clearance,clearane_dist);
prm_adj_with_clearance = [prm_adj_list(:,1:2) configs_clearance(prm_adj_list(:,2))];

% remove edges containing invalid configs
invalid_adj = ismember(prm_adj_with_clearance(:,1:2),invalid_nodes);
[ridx,~] = find(invalid_adj == 1);
ridx = unique(ridx);
valid_prm_adj = prm_adj_with_clearance(:,1:3);
valid_prm_adj(ridx,:) = [];

% prm_adj_with_clearance = [[valid_prm_adj valid_adj_clearance_value];[total_configs total_configs 0]];
prm_adj_with_clearance = [[valid_prm_adj];[total_configs total_configs 0]];

% create prm graph using distances between configs
G_prm = digraph(prm_adj_with_clearance(:,1),prm_adj_with_clearance(:,2),prm_adj_with_clearance(:,3));

% % calculate mincut nodes
% Ds = prm_adj_with_clearance;
% Ds(:,3) = 1;
% As = spconvert(Ds);
% A = full(As);  
% A= A + A';
% [costMat minCutNodes]= PRMAnalysisSparse(A,startNodes,goalNodes,total_config);
% max_clearance = max(configs_clearance(minCutNodes(:,1)));

% find path between start and goal config that obeys clearance
[path_distance] = distances(G_prm,startNodes,goalNodes);
minValue = min(path_distance(:));
[snode_idx, gnode_idx] = find(path_distance == minValue);

s_node = startNodes(snode_idx);
g_node = goalNodes(gnode_idx);

path= [];
min_path_clearance = -1;
% while(numel(path) == 0 || (min_path_clearance < clearane_dist))
while(numel(path) == 0 )
%     s_node = startNodes(randi(numel(startNodes))) % 68 %18 104 %
%     g_node = goalNodes(randi(numel(goalNodes))) % 2108 %2058 14066 %
    [path,path_distance] = shortestpath(G_prm,s_node(1),g_node(1));
    %     min_path_clearance = min(configs_clearance(path));
end
% plot valid and invalid configs
xCord = configs(:,1);
yCord = configs(:,2);
h = plot(G_prm,'XData',xCord,'yData',yCord);
hold on
% highlight(h,[path],'EdgeColor','r','LineWidth',5);
configs(path,1:3)
path_distance
highlight(h,[path],'NodeColor','k','MarkerSize',4)
plot(configs(path,1),configs(path,2),'r')
end

