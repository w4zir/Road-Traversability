        function [ output_args ] = roadRepair( input_args )
%Use rti data for road repair analysis
%   Detailed explanation goes here

adj_dir = '/home/khan/phd_ws/traversability/adjacency/village/';
config_dir =  '/home/khan/phd_ws/traversability/configs/village/';
pointcloud_dir =  '/home/khan/phd_ws/traversability/pointclouds/village/';

name_prefix = 'village';
vehicles = ['vehicle1';'vehicle2';'vehicle3';'vehicle4'];

% log data
mincut_log = [];
rti_log = [];

for cloud_id=1:6
    mc_log = [];
    for veh_id=1:size(vehicles,1)
        fprintf(strcat('cloud_id:' , int2str(cloud_id) , '\t veh_id' , int2str(veh_id), '\n'))
        
        % get vehicle name
        vehicle = vehicles(veh_id,:);
        % load vehicle parameters
        run('parameters_kitti.m')
        % read files
        adjFile = strcat(adj_dir,vehicles(veh_id,:),'/',name_prefix,int2str(cloud_id),'_adj');
        % adjFile
        configFile = strcat(config_dir,vehicles(veh_id,:),'/',name_prefix,int2str(cloud_id),'_conf');
        %         obsFile = strcat(pointcloud_dir,name_prefix,int2str(cloud_id),'_obs');
        Ds = importdata(adjFile,' ');
        configs = importdata(configFile,' ');
        %         obstacles = importdata(obsFile,' ');
        
        % read start, goal and total configs
        startNodes = find(configs(:,2) < min_y+2 & configs(:,3) == 90);
        startNodes = find(configs(:,2) < min_y+1.5);
        goalNodes = find(configs(:,2) > max_y-1.5);
        total_config = size(configs,1);
        
        % adjust indices according to matlab and get an undirected graph
        if (size(Ds,1) ~= 0)
            Ds = Ds(:,1:3); % remove angle ratio
            Ds(:,1:2) = Ds(:,1:2) +1;
            Ds(:,3) = configs(Ds(:,1),3);
            Ds = [Ds; [Ds(:,2) Ds(:,1) Ds(:,3)]];
            Ds = [Ds;[total_config total_config 0]];
            
            % convert into sparse matrix and then graph
            As = spconvert(Ds);
            A = full(As);
            
            % compute mincut
            [costMat minCutNodes]= PRMAnalysisSparse(A,startNodes,goalNodes,total_config);
            mincut = size(minCutNodes,1);
            mc_log = [mc_log; mincut];
        else
            mc_log = [mc_log; 0];
        end
    end
    mincut_log = [mincut_log mc_log];
end
max_val = max(mincut_log,[],2);
rti_log = mincut_log./repmat(max_val,1,size(mincut_log,2));

% plot commulative rti
avg_rti = sum(rti_log,1)/size(rti_log,1);
plot(avg_rti)

% save results
save('/home/khan/phd_ws/matlab_ws/RTI/logs/village.mat','mincut_log','rti_log','avg_rti')
end

