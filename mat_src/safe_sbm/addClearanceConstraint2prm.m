function [ adj_list_with_clearance ] = addClearanceConstraint2prm( adj_list_in,config_clearance, clearance_dist )
% Add clearance constraint to adjacency list.
%   Mark the distance to nodes that does not obey clearance constraint very
%   high and leave the others unchanged.

high_cost = 100;
adj_list_with_clearance = adj_list_in;

config_cleared_idx = find(config_clearance < clearance_dist);

[ai] = ismember(adj_list_in(:,2),config_cleared_idx);

adj_list_with_clearance(ai,3) = high_cost + adj_list_with_clearance(ai,3);


end

