function [config_clearance_value adj_list] = neighborBasedConfigsClearance(configs,min_t,max_t,theta_configs,valid_nodes,invalid_nodes )
%Compute clearance of each configuration.
%   Compute clearance of each configuration based on the distance from the
%   nearest invalid configuration.


total_configs = size(configs,1);
tire_width = 0.25;
nodes = [1:total_configs]';
t_res = (max_t-min_t)/(theta_configs-1);
adj_list = [];

for theta=min_t:t_res:max_t
    idx = find (configs(:,3) == theta);
    [from_nodes, to_nodes] = meshgrid(idx,idx);
    from_nodes = reshape(from_nodes,numel(from_nodes),1);
    to_nodes = reshape(to_nodes,numel(to_nodes),1);
    
    A_dist = round(sqrt(sum((configs(from_nodes,1:2)-configs(to_nodes,1:2)).^2,2)),2);
    idx = find(A_dist == tire_width | A_dist == round(sqrt(2)*tire_width,2));
    adj_list = [adj_list; [from_nodes(idx) to_nodes(idx) A_dist(idx)]];
end

% only include side wise neighbors
% t_adj = [configs(adj_list(:,1),1:3) configs(adj_list(:,2),1:2)];
% t_orientation = [round(180*atan2((t_adj(:,2)-t_adj(:,5)),(t_adj(:,1)-t_adj(:,4)))/pi) - t_adj(:,3)];
% idx = find(t_orientation ~= 0);

% adj_list = adj_list(idx,:);
% adj_list = [adj_list tire_width*ones(size(adj_list,1),1)];
adj_list = [adj_list; [total_configs total_configs 0]];
% 
% plot(configs(:,1),configs(:,2),'b*');
% hold on
% plot(configs(adj_list(:,1),1),configs(adj_list(:,1),2),'r*');
% plot(configs(adj_list(:,2),1),configs(adj_list(:,2),2),'g*');


% [to_nodes,from_nodes] = meshgrid(nodes,nodes);
% from_nodes = reshape(from_nodes,numel(from_nodes),1);
% to_nodes = reshape(to_nodes,numel(to_nodes),1);
% 
% % create neighbor matrix
% A = 0.5*ones(total_configs,total_configs);
% A_theta_diff = configs(from_nodes,3)-configs(to_nodes,3);
% idx = find(A_theta_diff ~= 0);
% a_idx = (to_nodes(idx)-1)*total_configs + from_nodes(idx);
% A(a_idx) = 0;
% 
% A_dist = sqrt(sum((configs(from_nodes,1:2)-configs(to_nodes,1:2)).^2,2));
% idx = find(A_dist ~= 0.5);
% a_idx = (to_nodes(idx)-1)*total_configs + from_nodes(idx);
% A(a_idx) = 0;

% get neighbor ids and find neighboring edge weights
% neighbor_idx = (neighbors(:,2)-1)*x_config_count*theta_config_count + (neighbors(:,1)-1)*theta_config_count+neighbors(:,3);
% edge_weights = sqrt(sum((configs(node_idx,1:2) - configs(neighbor_idx,1:2)).^2,2));

% create neighbor based graph
% n_adj_list = [node_idx neighbor_idx edge_weights; [total_configs total_configs 0]];
G_neighbor = digraph(adj_list(:,1),adj_list(:,2),adj_list(:,3));

% find and set clearance value of nodes
config_clearance_value = zeros(total_configs,1);
config_clearance_value(invalid_nodes,:) = 0.001;
obs_distances = distances(G_neighbor,valid_nodes,invalid_nodes);
clearance_dist = min(obs_distances,[],2);
% clearance_value = 1./clearance_dist;

config_clearance_value(valid_nodes,:) = round(clearance_dist,2);

end

