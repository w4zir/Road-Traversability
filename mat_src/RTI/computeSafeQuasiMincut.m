function [ output_args ] = computeSafeQuasiMincut( input_args )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
close all

load('logs/road_clear_pos_rand_01_10.mat');
safeMincutLog = [];
safeMincutCount = [];

frameCount = max(unique(minCutLog(:,5)));

for file_id=1:frameCount
    file_id
    
    %     idx = find(minCutLog(:,5) == file_id & minCutLog(:,4) == 90);
    idx = find(minCutLog(:,5) == file_id);
    %     mcidx = minCutLog(idx,1);
    mc_configs = minCutLog(idx,1:4);
    clusters = clusterdata(mc_configs(:,2:3),'criterion','distance','cutoff',0.4);
    [c_freq,c_id]=hist(clusters,unique(clusters));
    [~,max_c_idx] = max(c_freq);
    max_c_id = c_id(max_c_idx);
    cidx = find(clusters==max_c_id);
    
    safeMincut = mc_configs(cidx,:);
    safeMincutLog = [safeMincutLog; [[safeMincut file_id*ones(size(safeMincut,1),1)]]];
    %     valididx = find(configs(:,5) == 1);
    %     valid = configs(valididx,:);
    %     plot(configs(:,1),configs(:,2),'r.')
    
    %     plot(valid(:,1),valid(:,2),'g.');
    %     plot(mc_configs(:,2),mc_configs(:,3),'bs');
    %     hold on
    %     plot(safeMincut(:,2),safeMincut(:,3),'ks');
    %     axis([-4 4 -11 11])
    %     hold off
    safeMincutCount = [safeMincutCount; [size(safeMincut,1) file_id]];
end
% maxSafeValue = max(safeMincutCount(:,1));

save('/home/wazir/phd_ws/matlab_ws/RTI/logs/road_clear_pos_rand_01_10.mat','safeMincutCount','mincutCount','safeMincutLog','minCutLog','connCompsLog','startNodesLog','goalNodesLog','configsLog','adjacencyLog','obstacleLog')

end

