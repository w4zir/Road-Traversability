function [ output_args ] = comparePRMs( input_args )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

directory = '/home/khan/phd_ws/traversability/adjacency/synthetic/road_obs'
PRMdirectory = '/home/khan/phd_ws/traversability/prm/synthetic/road_obs'
x_config_cout = 17;
y_config_cout = 25;
theta_config_count = 5;
total_config = x_config_cout * y_config_cout * theta_config_count;

startNodes = [1:x_config_cout*theta_config_count]';
goalNodes = [total_config - x_config_cout*theta_config_count + 1:1:total_config]';

file1 = strcat(directory,'42_adj');
file2 = strcat(directory,'23_adj');
file3 = strcat(PRMdirectory,'42_prm');

D1 = importdata(file1,' ');
D1(:,1:2) = D1(:,1:2) +1;
% D1 = [D1; [D1(:,2) D1(:,1) D1(:,3)]];
D1 = [D1;[total_config total_config 0]];
A1 = spconvert(D1);
G1 = digraph(A1);

D2 = importdata(file2,' ');
D2(:,1:2) = D2(:,1:2) +1;
% D2 = [D2; [D2(:,2) D2(:,1) D2(:,3)]];
D2 = [D2;[total_config total_config 0]];
A2 = spconvert(D2);
G2 = digraph(A2);

D3 = importdata(file3,' ');

path_count = 1000;

valid_paths = 0;
invalid_paths = 0;
perc_valid = 0;
for i=1:path_count
    i
    cur_node = startNodes(randi(length(startNodes)));
    % rand_goal = t_list(randi(length(t_list)));
    %     cur_node = rand_start;
    path = [];
    terminate = 0;
    while(~terminate)
        neighbor_nodes =  successors(G1,cur_node);
        if(length(neighbor_nodes)==0)
            %             terminate = 1;
            path = [];
            cur_node = startNodes(randi(length(startNodes)));
            continue;
            %             break;
        end
        new_node = neighbor_nodes(randi(length(neighbor_nodes)));
        path = [path; [cur_node new_node D3(new_node,:)]];
        cur_node = new_node;
        idx = find(goalNodes==cur_node);
        if(length(idx)~=0)
            terminate = 1;
            break;
        end
    end
    for j=1:size(path,1)
        neighbor_nodes =  successors(G2,path(j,1));
        idx = find(neighbor_nodes==path(j,2));
        if(length(idx)==0)
            invalid_paths = invalid_paths + 1;
            break;
        end
        if(j==size(path,1))
            valid_paths  = valid_paths + 1;
        end
    end
    perc_valid = valid_paths/(valid_paths + invalid_paths);
    
    %     paths = [paths; [cur_path' i*ones(length(cur_path),1)]];
end
valid_paths
invalid_paths
valid_paths + invalid_paths
perc_valid

end

