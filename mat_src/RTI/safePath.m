function [ output_args ] = safePath( input_args )
%Safe Path: Find a safe path in a graph.
%   Detailed explanation goes here
close all
directory = '/home/khan/phd_ws/traversability/adjacency/test/road_obs';
% prmDirectory =  '/home/khan/phd_ws/traversability/prm/synthetic/road_obs';

file_id = 47;

min_x = -2;
max_x = 2;
min_y = -3;
max_y = 3;
x_config_count = 17;
y_config_count = 25;
theta_config_count = 1;

total_config = x_config_count * y_config_count * theta_config_count;

startNodes = [1:x_config_count*theta_config_count]';
goalNodes = [total_config - x_config_count*theta_config_count + 1:1:total_config]';

if(file_id < 10)
    adjFile = strcat(directory,'0',int2str(file_id),'_adj');
else
    adjFile = strcat(directory,int2str(file_id),'_adj');
end
% prmFile = strcat(prmDirectory,int2str(file_id),'_prm');

Ds = importdata(adjFile,' ');

Ds(:,1:2) = Ds(:,1:2) +1;
Ds = [Ds; [Ds(:,2) Ds(:,1) Ds(:,3)]];
Ds = [Ds;[total_config total_config 0]];
As = spconvert(Ds);
A_prm = full(As);
G_rti = graph(A_prm);

nodes = [1:x_config_count*y_config_count]';
valid_nodes = unique(Ds(:,1));
invalid_nodes = setdiff(nodes,valid_nodes);

x_idx = [1:x_config_count];
y_idx = [1:y_config_count];

[y_nodes,x_nodes] = meshgrid(y_idx,x_idx);
x_nodes = reshape(x_nodes,1,numel(x_nodes));
y_nodes = reshape(y_nodes,1,numel(y_nodes));

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
G = graph(A_adj);

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

xCord = reshape(xCord',1,x_config_count*y_config_count);
yCord = reshape(yCord',1,x_config_count*y_config_count);
% plot(G_dist,'XData',xCord,'yData',yCord);
% hold on;
node_val = 2000-config_obs_dist(:,1);
% minVal = min(min(node_val));
% minIdx = find(node_val>0 & node_val~=1000);
% node_val(minIdx) = node_val(minIdx) -minVal+1;
p = plot(G_rti,'XData',xCord,'yData',yCord,'NodeLabel',node_val);
axis([0 18 0 26]);
xlabel('Road width','interpreter','latex');
ylabel('Road length','interpreter','latex');
hold on
% find shortest path
for i=1:50
    s_node = startNodes(randi(17))
    g_node = goalNodes(randi(17))
%     s_node = 14;
%     g_node = 421;
    [path,d] = shortestpath(G_dist,s_node,g_node,'method','acyclic')
    
    highlight(p,path,'EdgeColor','g','LineWidth',3)
end



hold on;

sn_x_cord = mod(startNodes,x_config_count+1);
sn_y_cord = ceil(startNodes/y_config_count);
gn_x_cord = mod(goalNodes,x_config_count);
gn_x_cord(x_config_count) = x_config_count;
gn_y_cord = ceil(goalNodes/x_config_count);
% 
plot(sn_x_cord,sn_y_cord,'o','MarkerSize',7,'MarkerFaceColor','b');
plot(gn_x_cord,gn_y_cord,'o','MarkerSize',7,'MarkerFaceColor','g');
% 

% % hold on
% plot(mc_x(:,1),mc_y(:,1),'s','MarkerSize',7,'MarkerFaceColor','r');
% 
% 
% 

A = A_prm + A_prm';
[costMat minCutNodes]= PRMAnalysisSparse(A,startNodes,goalNodes,total_config);
mc_x = mod(minCutNodes,x_config_count);
mc_y = ceil(minCutNodes/x_config_count);

plot(mc_x(:,1),mc_y(:,1),'s','MarkerSize',7,'MarkerFaceColor','r');

legend('Start Nodes','Goal Nodes','Mincut nodes')

end

