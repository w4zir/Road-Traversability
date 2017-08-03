function [ output_args ] = toySafePaths( input_args )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

close all

x_config_count = 7;
y_config_count = 9;
theta_config_count = 1;

total_config = x_config_count * y_config_count * theta_config_count;

startNodes = [1:x_config_count*theta_config_count]';
goalNodes = [total_config - x_config_count*theta_config_count + 1:1:total_config]';


nodes = [1:x_config_count*y_config_count]';

x_idx = [1:x_config_count];
y_idx = [1:y_config_count];

[y_nodes,x_nodes] = meshgrid(y_idx,x_idx);
x_nodes = reshape(x_nodes,1,numel(x_nodes));
y_nodes = reshape(y_nodes,1,numel(y_nodes));

% PRM graph
x_nghr = [x_nodes-1 x_nodes x_nodes+1];
y_nghr = [y_nodes+1 y_nodes+1 y_nodes+1];
node_idx = repmat([1:length(x_nodes)]',3,1);

neighbors = [x_nghr' y_nghr'];
[xI yI] = find(x_nghr < 1 | x_nghr > x_config_count);
[xII yII] = find(y_nghr < 1 | y_nghr > y_config_count);
xidx = [yI  yII];
neighbors(xidx,:) = [];
node_idx(xidx,:) = [];

neighbor_idx = (neighbors(:,2)-1)*x_config_count + neighbors(:,1);

% remove invalid nodes connections
invalid_nodes_idx = [[1:x_config_count:total_config]'; [x_config_count:x_config_count:total_config]'; 2+2*x_config_count; 3*x_config_count-1];
% [xIdx yIdx] = find(neighbors == invalid_nodes_idx);
[ia,ib] = ismember([node_idx neighbor_idx],invalid_nodes_idx);
[ix iy] = find(ia==1);
neighbor_idx(ix,:) = [];
node_idx(ix,:) = [];

edge_weights = ones(size(neighbor_idx));

conn_list = [node_idx neighbor_idx edge_weights; [total_config total_config 0]];

% neighbors = [neighbors ones(length(neighbors),1)];
% neighbors = [neighbors; total_config total_config 0];

Ds = spconvert(conn_list);
A_prm = full(Ds);
G_prm = digraph(A_prm);
% plot(G_prm);

valid_nodes = unique(conn_list(:,1:2));
last_node_idx = find(valid_nodes==total_config);
valid_nodes(last_node_idx,:) = [];

invalid_nodes = setdiff(nodes,valid_nodes);

x_neighbors = [x_nodes-1 x_nodes x_nodes+1 x_nodes];
y_neighbors = [y_nodes y_nodes-1 y_nodes y_nodes+1];
node_idx = repmat([1:length(x_nodes)]',4,1);

neighbors = [x_neighbors' y_neighbors'];
[xI yI] = find(x_neighbors < 1 | x_neighbors > x_config_count);
[xII yII] = find(y_neighbors < 1 | y_neighbors > y_config_count);

% neighbors = [neighbors];

xidx = [yI  yII];
neighbors(xidx,:) = [];
node_idx(xidx,:) = [];

neighbor_idx = (neighbors(:,2)-1)*x_config_count + neighbors(:,1);

edge_weights = ones(size(neighbor_idx));

adj_list = [node_idx neighbor_idx edge_weights; [total_config total_config 0]];

% neighbors = [neighbors ones(length(neighbors),1)];
% neighbors = [neighbors; total_config total_config 0];

As_adj = spconvert(adj_list);
A_adj = full(As_adj);
G_adj = graph(A_adj);

config_obs_dist = zeros(total_config,2);
config_obs_dist(invalid_nodes,:) = config_obs_dist(invalid_nodes,:)+repmat([-1000 1],length(invalid_nodes),1);

for i=1:x_config_count
    dist = A_adj^i;
    obs_dist = dist(valid_nodes,invalid_nodes);
    [ix iy] = find(obs_dist~=0);
    xIdx = ix;%unique(ix);
    config_obs_dist(valid_nodes(xIdx),:) = config_obs_dist(valid_nodes(xIdx),:)+repmat([i 1],length(xIdx),1);
    valid_nodes(xIdx,:) = [];
    if(length(valid_nodes)==0)
        break;
    end
end
config_obs_dist = 2000-config_obs_dist;
config_obs_dist_rep = repmat(config_obs_dist(:,1)',total_config,1);

A_dist = A_prm.*config_obs_dist_rep;
G_dist = digraph(triu(A_dist));

nodes_x = [1:x_config_count];
nodes_y = [1:y_config_count];
[xCord yCord] = meshgrid(nodes_x,nodes_y);
% %
% % % xRes = (max_x - min_x)/(x_config_count-1);
% % % yRes = (max_y - min_y)/(y_config_count-1);
% % % xC = repmat(min_x:xRes:max_x,1,y_config_count);
% % % yC = reshape(repmat(min_y:yRes:max_y,x_config_count,1),1,length(xC));
% %
xCord = reshape(xCord',1,x_config_count*y_config_count);
yCord = reshape(yCord',1,x_config_count*y_config_count);
% plot(G_dist,'XData',xCord,'yData',yCord);
% hold on;
node_val = 2000-config_obs_dist(:,1);
% minVal = min(min(node_val));
% minIdx = find(node_val>0 & node_val~=1000);
% node_val(minIdx) = node_val(minIdx) -minVal+1;
p = plot(G_prm,'XData',xCord,'yData',yCord,'NodeLabel',node_val);
axis([0.5 x_config_count+0.5 0.5 y_config_count+0.5]);
xlabel('Road width','interpreter','latex');
ylabel('Road length','interpreter','latex');
hold on
% find shortest path
for i=1:10
    s_node = startNodes(randi(x_config_count))
    g_node = goalNodes(randi(x_config_count))
    %     s_node = 14;
    %     g_node = 421;
    [path,d] = shortestpath(G_dist,s_node,g_node,'method','acyclic')
    
    highlight(p,path,'EdgeColor','g','LineWidth',3)
end

A = A_prm + A_prm';
[costMat minCutNodes]= PRMAnalysisSparse(A,startNodes,goalNodes,total_config);

sn_x_cord = mod(startNodes,x_config_count+1);
sn_y_cord = ceil(startNodes/y_config_count);
gn_x_cord = mod(goalNodes,x_config_count);
gn_x_cord(x_config_count) = x_config_count;
gn_y_cord = ceil(goalNodes/y_config_count);

plot(sn_x_cord,sn_y_cord,'o','MarkerSize',7,'MarkerFaceColor','b');
plot(gn_x_cord,gn_y_cord,'o','MarkerSize',7,'MarkerFaceColor','g');

mc_x = mod(minCutNodes,x_config_count);
mc_y = ceil(minCutNodes/x_config_count);
% hold on
plot(mc_x(:,1),mc_y(:,1),'s','MarkerSize',7,'MarkerFaceColor','r');



legend('Start Nodes','Goal Nodes','Mincut nodes')
% plot(G);

end

