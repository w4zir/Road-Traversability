function [ output_args ] = percentageRTI( input_args )
%Compute mincut/RTI by removing a percentage of nodes
%   Detailed explanation goes here

close all;

%% ==================== Part 1: Vehicle info and general parameters ==============
total_configs = 14175;
perc_node_remove = 10;

pc_count = 60;
random_configs = 25;

%% ==================== Part 2: Load Data ====================
load('rti_analysis2.mat');

%% =============== Part 3: Remove nodes and compute mincut ============
mincutCount = [mincutCount zeros(size(mincutCount,1),1)];

for perc = 10:10:90
    for rconfig_id=1:25
        % ==================== Get random nodes ====================
        rand_nodes_count = round(perc*total_configs/100);
        rand_nodes = randi(total_configs,rand_nodes_count,1);
        for pc_no=1:60
            fprintf ('perc:%d, rconfig_id: %d and pc_no: %d \n',perc, rconfig_id,pc_no)
            % get frame configs
            idx = find(configsLog(:,7) == pc_no & configsLog(:,8) == rconfig_id);
            configs = configsLog(idx,1:6);
            total_configs = size(configs,1);
            
            % get adjacency data
            idx = find(adjacencyLog(:,4) == pc_no & adjacencyLog(:,5) == rconfig_id);
            adjacency = adjacencyLog(idx,1:3);
            
            % get start nodes
            idx = find(startNodesLog(:,2) == pc_no & startNodesLog(:,3) == rconfig_id);
            startNodes = startNodesLog(idx,1);
            
            % get goal nodes
            idx = find(goalNodesLog(:,2) == pc_no & goalNodesLog(:,3) == rconfig_id);
            goalNodes = goalNodesLog(idx,1);
            
            % =============== remove nodes =======
            idx = ismember(adjacency(:,1:2),rand_nodes);
            adjacency(idx(:,1),3) = 0;
            adjacency(idx(:,2),3) = 0;
            
            idx = ismember(startNodes,rand_nodes);
            startNodes(idx) = [];
            idx = ismember(goalNodes,rand_nodes);
            goalNodes(idx) = [];
            
            % =============== Compute Mincut =======
            adjacency = [adjacency; [adjacency(:,2) adjacency(:,1) adjacency(:,3)]];
            As = spconvert(adjacency);
            A = full(As);
            [connComps, minCutNodes]= PRMAnalysisSparse(A,startNodes,goalNodes,total_configs);
            
            cost = 0;
            if(length(connComps)==0)
                cost = 0;
            else
                cost = sum(connComps(:,4));
            end
            mincutCount = [mincutCount; [cost pc_no rconfig_id perc]];
        end
    end
end
save('/home/khan/phd_ws/matlab_ws/RTI/logs/perc_rti_analysis2.mat','mincutCount','-v7.3')
end

