function [ path_length_log, path_safety_log, paths_log, x_nodes, y_nodes ] = safePathService(prm_adj_list, configs, min_x, max_x, ...
min_y, max_y, min_t, max_t, x_config_count, y_config_count, theta_config_count)
%Safe Path: Find a safe path in a graph.
%   Detailed explanation goes here

total_config = x_config_count * y_config_count * theta_config_count;

startNodes = [1:x_config_count*theta_config_count]';
goalNodes = [total_config - x_config_count*theta_config_count + 1:1:total_config]';

% calculated prm distances
prm_adj_list(:,1:2) = prm_adj_list(:,1:2) +1;
prm_nodes_coord = configs(prm_adj_list(:,1),1:2);
prm_neighbors_coord = configs(prm_adj_list(:,2),1:2);
prm_distances = sqrt(sum((prm_nodes_coord - prm_neighbors_coord).^2,2));

% find valid and invalid node ids
nodes = [1:x_config_count*y_config_count*theta_config_count]';
valid_nodes = find(configs(:,5)==1);
invalid_nodes = find(configs(:,5)==0);

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
neighbor_idx = (neighbors(:,2)-1)*x_config_count*theta_config_count + (neighbors(:,1)-1)*theta_config_count+neighbors(:,3);
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

% create prm graph ids
prm_node_idx = [(prm_adj_list(:,1)-1)*total_config + prm_adj_list(:,2); prm_adj_list(:,3)];

% find shortest path
s_node = 115 %startNodes(randi(numel(startNodes))) % 68 %18 
g_node = 14066 %goalNodes(randi(numel(goalNodes))) % 2108 %2058
path_length_log = [];
path_safety_log = [];
paths_log = [];
path = [];
for delta=0:0.1:1
    prm_edge_value = (prm_clearance_value.^delta).*(prm_distances).^(1-delta);
    prm_adj = [[prm_adj_list(:,1:2) prm_edge_value];[total_config total_config 0]];
    As = spconvert(prm_adj);
    G_prm = digraph(As);
    
    [path,path_distance] = shortestpath(G_prm,s_node,g_node);
    paths_log = [paths_log; [path' delta*ones(numel(path),1)]];
    if(numel(path) == 0)
        break;
    end
    node_clearance_distances = 1./config_clearance_value;
    path_safety = node_clearance_distances(path);
    path_safety_value = sum(path_safety);
    path_safety_log = [path_safety_log; path_safety_value];
    
    path_adj_list = [path(1:numel(path)-1); path(2:numel(path))]';
    path_node_idx = (path_adj_list(:,1)-1)*total_config + path_adj_list(:,2);
    [C,ia,ib] = intersect(prm_node_idx,path_node_idx);
    path_length = prm_distances(ia);
    path_length_value = sum(path_length);
    path_length_log = [path_length_log; path_length_value];
end


end

