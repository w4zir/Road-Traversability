close all
clear all
clc

%% vehicle info
x_min = -3;
x_max = 3;
y_min = -10;
y_max = 10;
x_config_count = 25;
y_config_count = 81;
theta_config_count = 7;
costLog = [];
costMatLog = [];

start_n_goal_nodes_factore = 1;

%% load data from files
adjDirectory = '/home/az/git_repos/phd/road-traversability/data/adjacency/clearance3/vehicle2/clear';
configDirectory =  '/home/az/git_repos/phd/road-traversability/data/configurations/clearance3/vehicle2/clear';
file_id = 1;
adjFile = strcat(adjDirectory,int2str(file_id),'_adj');
configFile = strcat(configDirectory,int2str(file_id),'_conf');

Ds = importdata(adjFile,' ');
configs = importdata(configFile,' ');

% startNodes = find(configs(:,2) < y_min+2 & configs(:,3) == 90);
startNodes = find(configs(:,2) < y_min+start_n_goal_nodes_factore);
goalNodes = find(configs(:,2) > y_max-start_n_goal_nodes_factore);
total_config = size(configs,1);
valid_nodes = find(configs(:,5) == 1);
invalid_nodes = find(configs(:,5) == 0)
node_weights = configs(valid_nodes,6);
