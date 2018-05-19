function [ output_args ] = safeQuasiPathsWithClearance( input_args )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

close all
adj_directory = '/home/khan/phd_ws/traversability/adjacency/long_roads_quasi2/road_long';
config_directory = '/home/khan/phd_ws/traversability/configs/long_roads_quasi2/road_long';

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
% configs_clearance = configs(:,6);

% find clearance based local maximas
prm_clr = [configs_clearance(cl_adj_list(:,1)) configs_clearance(cl_adj_list(:,2))];
idx = find(prm_clr(:,1) < prm_clr(:,2));
non_local_max_nodes = cl_adj_list(idx,1);
local_max_nodes =  setdiff(valid_nodes,non_local_max_nodes);

% get prm adjacency list along with their distances
prm_adj_without_clearance = [[prm_adj_list(:,1:2) prm_distances];[total_configs total_configs 0]];

% adjust prm adjacency list by adding clearance constraint
prm_adj_with_clearance = addClearanceConstraint2prm(prm_adj_without_clearance,configs_clearance,clearane_dist);


% create prm graph using distances between configs
G_prm = digraph(prm_adj_with_clearance(:,1),prm_adj_with_clearance(:,2),prm_adj_with_clearance(:,3));

% calculate mincut nodes
Ds = prm_adj_with_clearance;
Ds(:,3) = 1;
As = spconvert(Ds);
A = full(As);
A= A + A';
[costMat minCutNodes]= PRMAnalysisSparse(A,startNodes,goalNodes,total_configs);
max_clearance = max(configs_clearance(minCutNodes(:,1)));

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
figure
valid = find(configs(:,5)==1);
invalid = find(configs(:,5)==0);
x_prm = configs(:,1);
y_prm = configs(:,2);
t_prm = configs(:,3);
u_prm = cosd(t_prm);
v_prm = sind(t_prm);
quiver(x_prm(valid),y_prm(valid),u_prm(valid),v_prm(valid),0.5,'g')
hold on
quiver(x_prm(invalid),y_prm(invalid),u_prm(invalid),v_prm(invalid),0.5,'r')

% plot paths
colors = ['r';'k';'b'];
marker_types = ['-o';'-+';':*']
x_path = configs(path,1);
y_path = configs(path,2);
plot(x_path,y_path,marker_types(1,:),'LineWidth',1,'MarkerSize',3,'MarkerFaceColor',colors(2));

% mark configurations that fail clearance constraint as red star
config_unsafe_idx = find(configs_clearance(path) < clearane_dist);
x_unsafe = configs(path(config_unsafe_idx),1);
y_unsafe = configs(path(config_unsafe_idx),2);
plot(x_unsafe,y_unsafe,'r*');

% plot mincut
mc_x = configs(minCutNodes(:,1),1);
mc_y = configs(minCutNodes(:,1),2);
plot(mc_x,mc_y,'s','MarkerSize',3,'MarkerFaceColor','b');

% quiver(x_prm(minCutNodes(:,1)),y_prm(minCutNodes(:,1)),u_prm(minCutNodes(:,1)),v_prm(minCutNodes(:,1)),0.5,'b')

% x_prm = configs(local_max_nodes,1);
% y_prm = configs(local_max_nodes,2);
% t_prm = configs(local_max_nodes,3);
% u_prm = cosd(t_prm);
% v_prm = sind(t_prm);
% quiver(x_prm,y_prm,u_prm,v_prm,0.5,'b')
% plot(configs(local_max_nodes,1),configs(local_max_nodes,2),'bs','MarkerSize',3);

path
% configs(path,1:3)
path_distance

min(configs_clearance(minCutNodes(:,1)))
max(configs_clearance(minCutNodes(:,1)))

size(minCutNodes,1)
% configs(minCutNodes(:,1),1:4)
end

