function [ output_args ] = toyPRM( input_args )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

close all

x_config_count = 5;
y_config_count = 7;
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
% invalid_nodes_idx = [[1:x_config_count:total_config]'; [x_config_count:x_config_count:total_config]'; 2+2*x_config_count; 3*x_config_count-1];
% invalid_nodes_idx = [22;24;23;29;31;30;36;38;37];
% invalid_nodes_idx = [24;25;26;31;32;33;38;39;40];
invalid_nodes_idx = [16;20];
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
% G_prm = digraph(A_prm);
% plot(G_prm);

Adj_list = [conn_list; [conn_list(:,2) conn_list(:,1) conn_list(:,3)]];
Adj_prm = full(spconvert(Adj_list));
G_prm = graph(Adj_prm);

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

nodes_x = [1:x_config_count];
nodes_y = [1:y_config_count];
[ yCord xCord] = meshgrid(nodes_y,nodes_x);

p = plot(G_prm,'XData',xCord(:),'yData',yCord(:),'NodeLabel','','LineWidth',2,'MarkerSize', 6);
axis([0.5 x_config_count+0.5 0.5 y_config_count+0.5]);
% xlabel('Road width','interpreter','latex');
% ylabel('Road length','interpreter','latex');
hold on

% sn_x_cord = mod(startNodes,x_config_count+1);
% sn_y_cord = ceil(startNodes/y_config_count);
% gn_x_cord = mod(goalNodes,x_config_count);
% gn_x_cord(x_config_count) = x_config_count;
% gn_y_cord = ceil(goalNodes/y_config_count);
sn_x_cord = xCord(startNodes);
sn_y_cord = yCord(startNodes);
gn_x_cord = xCord(goalNodes);
gn_y_cord = yCord(goalNodes);

plot(sn_x_cord,sn_y_cord,'o','MarkerSize',8,'MarkerFaceColor','k');
plot(gn_x_cord,gn_y_cord,'o','MarkerSize',8,'MarkerFaceColor','g');

A = A_prm + A_prm';
[costMat minCutNodes]= PRMAnalysisSparse(A,startNodes,goalNodes,total_config);

mc_x = xCord(minCutNodes(:,1));
mc_y = yCord(minCutNodes(:,1));
% hold on
plot(mc_x(:,1),mc_y(:,1),'s','MarkerSize',8,'MarkerFaceColor','r');

invalid_x = xCord(invalid_nodes_idx);
invalid_y = yCord(invalid_nodes_idx);
plot(invalid_x,invalid_y,'o','MarkerSize',6,'MarkerFaceColor','r');

yticklabels({});
xticklabels({});
xticks([])
yticks([])
% Here we preserve the size of the image when we save it.
width = 4;
height = 6;
set(gcf,'InvertHardcopy','on');
set(gcf,'PaperUnits', 'inches');
papersize = get(gcf, 'PaperSize');
left = (papersize(1)- width)/2;
bottom = (papersize(2)- height)/2;
myfiguresize = [left, bottom, width, height];
set(gcf,'PaperPosition', myfiguresize);

% Save the file as PNG
print('/home/khan/Pictures/matlab_imgs/toy_prm.png','-dpng','-r150');

% legend('Intermediate Nodes','Start Nodes','Goal Nodes','Mincut nodes')
% set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
% plot(G);

end

