function [ output_args ] = compareQuasiClearance( input_args )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

close all
adj_directory = '/home/khan/phd_ws/traversability/adjacency/long_roads_quasi/tmp/road_long';
config_directory = '/home/khan/phd_ws/traversability/configs/long_roads_quasi/tmp/road_long';

file_id = 24;

min_x = -3;
max_x = 3;
min_y = -10;
max_y = 10;
min_t = 45;
max_t = 135;
x_config_count = 25;
y_config_count = 81;
theta_config_count = 7;
wheelbase = 1.78;
clearane_dist = 0.75;
high_cost = 100;


adjFile = strcat(adj_directory,int2str(file_id),'_adj');
configFile = strcat(config_directory,int2str(file_id),'_conf');

% Read prm and configuration files
prm_adj_list = importdata(adjFile,' ');
configs = importdata(configFile,' ');

% total configs and start and goal nodes
total_configs = size(configs,1);
startNodes = find(configs(:,2) < min_y+0.5);
goalNodes = find(configs(:,2) > max_y-0.5);

% calculated prm distances
prm_adj_list(:,1:2) = prm_adj_list(:,1:2) +1;
prm_nodes_coord = configs(prm_adj_list(:,1),1:3);
prm_neighbors_coord = configs(prm_adj_list(:,2),1:3);
xy_diff = prm_nodes_coord(:,1:2) - prm_neighbors_coord(:,1:2);
theta_diff = abs(prm_nodes_coord(:,3) - prm_neighbors_coord(:,3))*pi./180;
prm_distances = sqrt(sum((xy_diff).^2,2) + wheelbase^2 * (theta_diff).^2);

% find clearance of each node using neighbor invalid nodes
% 1. find valid and invalid node ids
nodes = [1:total_configs]';
valid_nodes = find(configs(:,5)==1);
invalid_nodes = find(configs(:,5)==0);


% 2. call the function to compute clearance of each config
[configs_clearance cl_adj_list] = neighborBasedConfigsClearance(configs,min_t,max_t,theta_config_count,valid_nodes,invalid_nodes);

valid_neighbor_based_clearance = configs_clearance(valid_nodes);
valid_actual_clearance = configs(valid_nodes,6);

clearance_diff = abs(valid_neighbor_based_clearance - valid_actual_clearance);

diff_mean = mean(clearance_diff)
diff_std = std(clearance_diff)

boxplot(clearance_diff)

end

