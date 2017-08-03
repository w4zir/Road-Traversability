function [ output_args ] = safePathSE2( input_args )
%Safe Path: Find a safe path in a graph.
%   Detailed explanation goes here
close all
adjDirectory = '/home/khan/phd_ws/traversability/adjacency/long_roads_quasi/road_long';
configDirectory =  '/home/khan/phd_ws/traversability/configs/long_roads_quasi/road_long';
% prmDirectory =  '/home/khan/phd_ws/traversability/prm/synthetic/road_obs';

file_id = 2;

delta = 1;

min_x = -3;
max_x = 3;
min_y = -10;
max_y = 10;
min_t = 60;
max_t = 120;
x_config_count = 25;
y_config_count = 81;
theta_config_count = 7;

total_config = x_config_count * y_config_count * theta_config_count;

startNodes = [1:x_config_count*theta_config_count]';
goalNodes = [total_config - x_config_count*theta_config_count + 1:1:total_config]';

% if(file_id < 10)
%     adjFile = strcat(directory,'0',int2str(file_id),'_adj');
%     prmFile = strcat(prm_directory,'0',int2str(file_id),'_prm');
% else
    adjFile = strcat(adjDirectory,int2str(file_id),'_adj');
    configFile = strcat(configDirectory,int2str(file_id),'_prt');
% end
% prmFile = strcat(prmDirectory,int2str(file_id),'_prm');

% Read prm and configuration files
prm_adj_list = importdata(adjFile,' ');
configs = importdata(configFile,' ');

% calculated prm distances
prm_adj_list(:,1:2) = prm_adj_list(:,1:2) +1;
prm_nodes_coord = configs(prm_adj_list(:,1),1:2);
prm_neighbors_coord = configs(prm_adj_list(:,2),1:2);
prm_distances = sqrt(sum((prm_nodes_coord - prm_neighbors_coord).^2,2));

% find valid and invalid node ids
nodes = [1:x_config_count*y_config_count*theta_config_count]';
valid_nodes = find(configs(:,4)==1);
invalid_nodes = find(configs(:,4)==0);

% create x,y and theta ids
x_idx = reshape(repmat([1:x_config_count],theta_config_count,1),x_config_count*theta_config_count,1);
y_idx = [1:y_config_count]';
t_values = [1:theta_config_count];

% find x,y and theta of nodes
[y_nodes,x_nodes] = meshgrid(y_idx,x_idx);
x_nodes = reshape(x_nodes,1,numel(x_nodes));
y_nodes = reshape(y_nodes,1,numel(y_nodes));
t_nodes = repmat(t_values,1,x_config_count*y_config_count);

% find neighbors of each nodes using 8 surrounding neighbors
x_neighbors = [x_nodes-1; x_nodes; x_nodes+1; x_nodes-1; x_nodes+1; x_nodes-1; x_nodes; x_nodes+1];
x_neighbors = reshape(x_neighbors,numel(x_neighbors),1);
y_neighbors = [y_nodes-1; y_nodes-1; y_nodes-1; y_nodes; y_nodes; y_nodes+1; y_nodes+1; y_nodes+1];
y_neighbors = reshape(y_neighbors,numel(y_neighbors),1);
t_neighbors = repmat(t_nodes,8,1);
t_neighbors = reshape(t_neighbors,numel(t_neighbors),1);

% remove neighbors outside boundary of road
node_idx = repmat(1:total_config,8,1);
node_idx = reshape(node_idx,numel(node_idx),1);
neighbors = [x_neighbors y_neighbors t_neighbors];
[xI yI] = find(x_neighbors < 1 | x_neighbors > x_config_count);
[xII yII] = find(y_neighbors < 1 | y_neighbors > y_config_count);
xidx = [xI;  xII];
neighbors(xidx,:) = [];
node_idx(xidx,:) = [];

% get neighbor ids and find neighboring edge weights
neighbor_idx = (neighbors(:,2)-1)*x_config_count*theta_config_count + (neighbors(:,1)-1)*theta_config_count+theta_config_count;
edge_weights = sqrt(sum((configs(node_idx,1:2) - configs(neighbor_idx,1:2)).^2,2));

% edge_weight_max = max(max(edge_weights));
% edge_weights = edge_weights/edge_weight_max;
% ones(size(neighbor_idx));

% create adjacency list and get the graph
adj_list = [node_idx neighbor_idx edge_weights; [total_config total_config 0]];
As_adj = spconvert(adj_list);
G_adj = digraph(As_adj);

% find and set clearance value of nodes
config_clearance_value = zeros(total_config,1);
config_clearance_value(invalid_nodes,:) = 0.001;
obs_distances = distances(G_adj,valid_nodes,invalid_nodes);
clearance_dist = min(obs_distances,[],2);
clearance_value = 1./clearance_dist;

config_clearance_value(valid_nodes,:) = clearance_value;
prm_clearance_value = config_clearance_value(prm_adj_list(:,2));

% calculate the weighted edges values and update prm graph
prm_edge_value = (prm_clearance_value.^delta).*(prm_distances).^(1-delta);
prm_adj = [[prm_adj_list(:,1:2) prm_edge_value];[total_config total_config 0]];
As = spconvert(prm_adj);
G_prm = digraph(As);

% find node coordinates
nodes_x = [1:x_config_count];
nodes_y = [1:y_config_count];
[xCord yCord] = meshgrid(nodes_x,nodes_y);
xCord = reshape(xCord',1,x_config_count*y_config_count);
yCord = reshape(yCord',1,x_config_count*y_config_count);

% find shortest path
path_count=0;
path = [];
while(numel(path)==0)
    %     s_node = startNodes(randi(numel(startNodes)))
    %     g_node = goalNodes(randi(numel(goalNodes)))
    s_node =38 %12 % 28;
    g_node =2095 %421 %2069 %total_config-37;
    [path,path_distance] = shortestpath(G_prm,s_node,g_node);
    %     highlight(p,path,'EdgeColor','g','LineWidth',3)
end
% path
% path_distance
node_clearance_distances = 1./config_clearance_value;
path_safety = node_clearance_distances(path);
path_safety_value = sum(path_safety)

prm_node_idx = [(prm_adj_list(:,1)-1)*total_config + prm_adj_list(:,2); prm_adj_list(:,3)];
path_adj_list = [path(1:numel(path)-1); path(2:numel(path))]';
path_node_idx = (path_adj_list(:,1)-1)*total_config + path_adj_list(:,2);
[C,ia,ib] = intersect(prm_node_idx,path_node_idx);
path_length = prm_distances(ia);
path_length_value = sum(path_length)


valid = find(configs(:,4)==1);
invalid = find(configs(:,4)==0);

x_prm = configs(:,1);
y_prm = configs(:,2);
t_prm = configs(:,3);
u_prm = cosd(t_prm);
v_prm = sind(t_prm);

quiver(x_prm(valid),y_prm(valid),u_prm(valid),v_prm(valid),0.5,'g')
hold on
quiver(x_prm(invalid),y_prm(invalid),u_prm(invalid),v_prm(invalid),0.5,'r')

x_coord = min_x:0.25:max_x;
y_coord = min_y:0.25:max_y;

x_path = x_coord(x_nodes(path));
y_path = y_coord(y_nodes(path));


hold on;
plot(x_path,y_path,'-o','MarkerSize',3,'MarkerFaceColor','b');

% txt = round((config_clearance_value*10))/10;
% prm_distances
% text(x_prm+0.1*u_prm,y_prm+0.1*v_prm,num2str(txt));

% sn_x_cord = x_nodes(1:x_config_count*theta_config_count);
% sn_y_cord = y_nodes(1:x_config_count*theta_config_count);
% gn_x_cord = x_nodes(total_config-x_config_count*theta_config_count+1:total_config);
% gn_y_cord = y_nodes(total_config-x_config_count*theta_config_count+1:total_config);
% %
% plot(sn_x_cord,sn_y_cord,'o','MarkerSize',7,'MarkerFaceColor','b');
% plot(gn_x_cord,gn_y_cord,'o','MarkerSize',7,'MarkerFaceColor','g');
%

% % hold on
% plot(mc_x(:,1),mc_y(:,1),'s','MarkerSize',7,'MarkerFaceColor','r');
%
%
%

% A = A_prm + A_prm';
% [costMat minCutNodes]= PRMAnalysisSparse(A,startNodes,goalNodes,total_config);
% mc_x = mod(minCutNodes,x_config_count);
% mc_y = ceil(minCutNodes/x_config_count);

% plot(mc_x(:,1),mc_y(:,1),'s','MarkerSize',7,'MarkerFaceColor','r');

% legend('Start Nodes','Goal Nodes','Mincut nodes')

end

