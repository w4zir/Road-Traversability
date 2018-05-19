function [ paths ] = randomPaths( s,t,s_list,g_list, path_count,invalid_nodes )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
paths = [];

valid_paths = 0;
invalid_paths = 0;
perc_valid = 0;
for i=1:path_count
    rand_start = s_list(randi(length(s_list)));
    % rand_goal = t_list(randi(length(t_list)));
    cur_node = rand_start;
    cur_path = [cur_node];
    terminate = 0;
    while(~terminate)
        neighbor_nodes = t(find(s==cur_node));
        if(length(neighbor_nodes)==0)
            terminate = 1;
            cur_path = [];
            break;
        end
        cur_node = neighbor_nodes(randi(length(neighbor_nodes)));
        cur_path = [cur_path cur_node];
        idx = find(g_list==cur_node);
         if(length(idx)~=0)
            terminate = 1;
            break;
         end
    end
    inv_common = intersect(cur_path,invalid_nodes);
    if(length(inv_common)>0)
        invalid_paths = invalid_paths + 1;
    else
        valid_paths  = valid_paths + 1;
    end
    perc_valid = valid_paths/(valid_paths + invalid_paths);
    
%     paths = [paths; [cur_path' i*ones(length(cur_path),1)]];
end
valid_paths
invalid_paths
perc_valid
end

