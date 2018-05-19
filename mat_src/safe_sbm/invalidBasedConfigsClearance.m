function [config_clearance_value adj_list] = invalidBasedConfigsClearance(configs,adj_list,prm_distances,valid_nodes,invalid_nodes )
%Compute clearance of each configuration.
%   Compute clearance of each configuration based on the distance from the
%   nearest invalid configuration.

total_configs = size(configs,1);

G = digraph(adj_list(:,1),adj_list(:,2),prm_distances);

% find and set clearance value of nodes
config_clearance_value = 100*ones(total_configs,1);
config_clearance_value(invalid_nodes,:) = 0.001;
obs_distances = distances(G,valid_nodes,invalid_nodes);
clearance_dist = min(obs_distances,[],2);
% clearance_value = 1./clearance_dist;

config_clearance_value(valid_nodes,:) = round(clearance_dist,2);

end

