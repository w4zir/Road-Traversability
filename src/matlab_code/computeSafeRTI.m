function [ output_args ] = computeSafeRTI( input_args )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
close all
% directory = '/home/khan/phd_ws/traversability/adjacency/narrow_passage/narrow_passage'
adj_dir = '/home/khan/phd_ws/traversability/adjacency/rsi/rand_01/road_rsi'
config_dir =  '/home/khan/phd_ws/traversability/configs/rsi/rand_01/road_rsi'
x_min = -4;
x_max = 4;
y_min = -10;
y_max = 10;
t_min = 60;
t_max = 120;
theta_config_count = 7;

load('logs/rti_compare_20.mat');
safeCostLog = [];

for file_id=1:3
    file_id
    configFile = strcat(config_dir,int2str(file_id),'_prt');
    configs = importdata(configFile,' ');
    
    idx = find(minCutLog(:,3) == file_id);
    mcidx = minCutLog(idx,1);
    mc_configs = configs(mcidx,:);
    clusters = clusterdata(mc_configs(:,1:2),'criterion','distance','cutoff',0.7);
    [c_freq,c_id]=hist(clusters,unique(clusters));
    [~,max_c_idx] = max(c_freq);
    max_c_id = c_id(max_c_idx);
    idx = find(clusters==max_c_id);
    
    c_configs = mc_configs(idx,:);
    
    valididx = find(configs(:,5) == 1);
    valid = configs(valididx,:);
    plot(configs(:,1),configs(:,2),'r.');
    hold on
    plot(valid(:,1),valid(:,2),'g.');
    plot(mc_configs(:,1),mc_configs(:,2),'bs');
    plot(c_configs(:,1),c_configs(:,2),'ks');
    axis([-4 4 -11 11])
    hold off
    safeMincutCount = [safeCostLog; size(c_configs,1)];
end
maxSafeValue = max(safeCostLog(:,1));

save('logs/rsi','safeMincutCount','mincutCount','minCutLog','connCompsLog')

end

