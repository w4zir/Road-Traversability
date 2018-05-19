function [ output_args ] = computeBatchQuasiRTI( input_args )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% directory = '/home/khan/phd_ws/traversability/adjacency/narrow_passage/narrow_passage'
adj_dir = '/home/khan/phd_ws/traversability/adjacency/rsi2/road_rsi'
config_dir =  '/home/khan/phd_ws/traversability/configs/rsi2/road_rsi'
pointcloud_dir =  '/home/khan/phd_ws/traversability/pointclouds/rsi2/road_rsi'

x_min = -3;
x_max = 3;
y_min = -10;
y_max = 10;
t_min = 45;
t_max = 135;
theta_config_count =7;

mincutCount = [];
safeMincutCount = [];
connCompsLog = [];
minCutLog = [];
safeMincutLog = [];
configsLog = [];
adjacencyLog = [];
obstacleLog = [];

startNodesLog = [];
goalNodesLog = [];


fileNameAdj = [];
for file_id=1:14
    file_id
    %     if (file_idx<10)
    %         fileNameAdj = strcat(directory,'0',int2str(file_idx),'_adj');
    %     else
    adjFile = strcat(adj_dir,int2str(file_id),'_adj');
    configFile = strcat(config_dir,int2str(file_id),'_conf');
    obsFile = strcat(pointcloud_dir,int2str(file_id),'_obs');
    %         fileNameAdj = strcat(directory,int2str(file_idx),'_5_adj');
    %     end
    
    
    Ds = importdata(adjFile,' ');
    configs = importdata(configFile,' ');
    obstacles = importdata(obsFile,' ');
    
    startNodes = find(configs(:,2) < y_min+2);
    goalNodes = find(configs(:,2) > y_max-2);
    total_config = size(configs,1);
    
    Ds(:,1:2) = Ds(:,1:2) +1;
    adjacency = Ds;
    adjacency = [adjacency;[total_config total_config 0]];
    
    Ds = [Ds; [Ds(:,2) Ds(:,1) Ds(:,3)]];
    Ds = [Ds;[total_config total_config 0]];
    
    if(length(Ds)==0)
        mincutCount = [Ds; 0];
        continue;
    end
    
    As = spconvert(Ds);
    A = full(As);
    
    [connComps, minCutNodes]= PRMAnalysisSparse(A,startNodes,goalNodes,total_config);
    
    %     if(file_idx > 48)
    %
    %     end
    
    if(length(connComps)==0)
        cost = 0;
    else
        cost = sum(connComps(:,4));
        minCutLog = [minCutLog; [minCutNodes(:,1) file_id*ones(size(minCutNodes,1),1)]];
        connCompsLog = [connCompsLog; connComps file_id*ones(size(connComps,1),1)];
         
        mc_configs = configs(minCutNodes(:,1),:);
        safeMincut = [];
        if (size(mc_configs,1) > 10)            
            clusters = clusterdata(mc_configs(:,1:2),'criterion','distance','cutoff',0.4);
            [c_freq,c_id]=hist(clusters,unique(clusters));
            [~,max_c_idx] = max(c_freq);
            max_c_id = c_id(max_c_idx);
            cidx = find(clusters==max_c_id);
            
            safeMincut = minCutNodes(cidx,1);
        else
            safeMincut = minCutNodes(:,1);
        end
        safeMincutLog = [safeMincutLog; [[safeMincut file_id*ones(size(safeMincut,1),1)]]];
        safeMincutCount = [safeMincutCount; [size(safeMincut,1)]];
    end
    mincutCount = [mincutCount; cost];
    configsLog = [configsLog; [configs file_id*ones(size(configs,1),1)]];
    adjacencyLog = [adjacencyLog; [adjacency file_id*ones(size(adjacency,1),1)]];
    
    startNodesLog = [startNodesLog; [startNodes file_id*ones(size(startNodes,1),1)]];
    goalNodesLog = [goalNodesLog; [goalNodes file_id*ones(size(goalNodes,1),1)]];
    
    obstacleLog = [obstacleLog; [obstacles file_id*ones(size(obstacles,1),1)]];
end

maxVal = max(mincutCount);
% mincutCount = mincutCount./maxVal;
save('/home/khan/phd_ws/matlab_ws/RTI/logs/rsi_2.mat','safeMincutCount','mincutCount','safeMincutLog','minCutLog','connCompsLog','startNodesLog','goalNodesLog','configsLog','adjacencyLog','obstacleLog','-v7.3')
% save('/home/wazir/phd_ws/matlab_ws/RTI/logs/clearance_roads_quasi','mincutCount','minCutLog','connCompsLog','startNodesLog','goalNodesLog','configsLog','adjacencyLog','obstacleLog')
figure
bar(mincutCount);


end

