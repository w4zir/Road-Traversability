function [ output_args ] = computeBatchRandomRTI2( input_args )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


close all;

%% ==================== Part 1: Vehicle info ====================
vehicles = ['vehicle1'; 'vehicle2'; 'vehicle3';'vehicle4';'vehicle5'];

%% load data from files
folder = 'analysis_new';
configDirectory =  '/home/khan/phd_ws/traversability/configs/';
adjDirectory = '/home/khan/phd_ws/traversability/adjacency/';
obsDirectory =  '/home/khan/phd_ws/traversability/pointclouds/';
file_prefix = 'road_rti';

x_min = -3;
x_max = 3;
y_min = -10;
y_max = 10;
t_min = 60;
t_max = 120;
theta_config_count =7;

mc_count = [];
safeMincutCount = [];
connCompsLog = [];
mc_log = [];
safeMincutLog = [];
configsLog = [];
adjacencyLog = [];
obstacleLog = [];

startNodesLog = [];
goalNodesLog = [];

fileNameAdj = [];
for v_id=2:2%size(vehicles,1)
    for conf_id=1:25
        for file_id=1:60
            configFile = strcat(configDirectory,folder,'/',vehicles(v_id,:),'/',file_prefix,int2str(file_id),'_',int2str(conf_id),'_100_conf');
            adjFile = strcat(adjDirectory,folder,'/',vehicles(v_id,:),'/',file_prefix,int2str(file_id),'_',int2str(conf_id),'_100_adj');
              Ds = importdata(adjFile,' ');
            configs = importdata(configFile,' ');
            %             obstacles = importdata(obsFile,' ');
            
            startNodes = find(configs(:,2) < y_min+2);
            goalNodes = find(configs(:,2) > y_max-2);
            total_configs = size(configs,1);
            
            Ds = Ds(:,1:3);
            Ds(:,1:2) = Ds(:,1:2) +1;
            
            Ds = [Ds; [Ds(:,2) Ds(:,1) Ds(:,3)]];
            Ds = [Ds;[total_configs total_configs 0]];
            
            if(length(Ds)==0)
                mc_count = [mc_count; [0 v_id file_id conf_id perc]];
                continue;
            end
            
            
            for perc=90:-10:10
            fprintf ('v_id: %d | conf_id: %d | file_id: %d | perc: %d \n', v_id,conf_id,file_id,perc)
            
             %             obsFile = strcat(obsDirectory,folder,'/',file_prefix,int2str(file_id),'_obs');
            % ==================== Get random nodes ====================
        rand_nodes_count = round((100-perc)*total_configs/100);
        rand_nodes = randi(total_configs,rand_nodes_count,1);
        
         % =============== remove nodes =======
            idx = ismember(Ds(:,1:2),rand_nodes);
            Ds(idx(:,1),3) = 0;
            Ds(idx(:,2),3) = 0;
            
            idx = ismember(startNodes,rand_nodes);
            startNodes(idx) = [];
            idx = ismember(goalNodes,rand_nodes);
            goalNodes(idx) = [];
          
            
            As = spconvert(Ds);
            A = full(As);
            
            [connComps, minCutNodes]= PRMAnalysisSparse(A,startNodes,goalNodes,total_configs);
            
            if(length(connComps)==0)
                cost = 0;
            else
                cost = sum(connComps(:,4));
                %                 mc_log = [mc_log; [minCutNodes(:,1) file_id*ones(size(minCutNodes,1),1) conf_id*ones(size(minCutNodes,1),1)]];
                %                 connCompsLog = [connCompsLog; connComps file_id*ones(size(connComps,1),1) conf_id*ones(size(connComps,1),1)];
                
                %                 mc_configs = configs(minCutNodes(:,1),:);
                %                 safeMincut = [];
                %                 if (size(mc_configs,1) > 10)
                %                     clusters = clusterdata(mc_configs(:,1:2),'criterion','distance','cutoff',0.4);
                %                     [c_freq,c_id]=hist(clusters,unique(clusters));
                %                     [~,max_c_idx] = max(c_freq);
                %                     max_c_id = c_id(max_c_idx);
                %                     cidx = find(clusters==max_c_id);
                %
                %                     safeMincut = minCutNodes(cidx,1);
                %                 else
                %                     safeMincut = minCutNodes(:,1);
                %                 end
                
                %                 safeMincutLog = [safeMincutLog; [[safeMincut file_id*ones(size(safeMincut,1),1) conf_id*ones(size(safeMincut,1),1)]]];
                %                 safeMincutCount = [safeMincutCount; [size(safeMincut,1) file_id conf_id]];
            end
            mc_count = [mc_count; [cost v_id file_id conf_id perc]];
            %             configsLog = [configsLog; [configs file_id*ones(size(configs,1),1) conf_id*ones(size(configs,1),1)]];
            %             adjacencyLog = [adjacencyLog; [adjacency file_id*ones(size(adjacency,1),1) conf_id*ones(size(adjacency,1),1)]];
            %
            %             startNodesLog = [startNodesLog; [startNodes file_id*ones(size(startNodes,1),1) conf_id*ones(size(startNodes,1),1)]];
            %             goalNodesLog = [goalNodesLog; [goalNodes file_id*ones(size(goalNodes,1),1) conf_id*ones(size(goalNodes,1),1)]];
            %
            %             obstacleLog = [obstacleLog; [obstacles file_id*ones(size(obstacles,1),1) conf_id*ones(size(obstacles,1),1)]];
            
            %         if(length(connComps)==0)
            %             cost = 0;
            %         else
            %             mc_log = [mc_log; [minCutNodes(:,1) configs(minCutNodes(:,1),1:3) file_id*ones(size(minCutNodes,1),1) conf_id*ones(size(minCutNodes,1),1)]];
            %             connCompsLog = [connCompsLog; connComps file_id*ones(size(connComps,1),1) conf_id*ones(size(connComps,1),1)];
            %             cost = sum(connComps(:,4));
            %         end
            %         mc_count = [mc_count; [cost file_id conf_id]];
            end
        end
    end
end
% maxVal = max(mc_count);
% mc_count = mc_count./maxVal;
%     save('logs/safe_roads_rand_20','mc_count','mc_log','connCompsLog')
%   save('/home/khan/phd_ws/matlab_ws/RTI/logs/road_repair.mat','safeMincutCount','mc_count','safeMincutLog','mc_log','connCompsLog','startNodesLog','goalNodesLog','configsLog','adjacencyLog','obstacleLog','-v7.3')
% figure
% bar(mc_count);
save('/home/khan/phd_ws/matlab_ws/RTI/logs/analysis_new_node_rem.mat','mc_count','-v7.3')

end

