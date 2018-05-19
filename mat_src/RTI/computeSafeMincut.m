function [ output_args ] = computeSafeMincut( input_args )
%Safe Mincut: Cluster mincut and find the largest component.
%   Detailed explanation goes here
close all

load('logs/tmp.mat');
safeMincutLog = [];
safeMincutCount = [];
for pc_no=1:4
    for file_id=1:20
        pc_no
        file_id
        
        idx = find(minCutLog(:,5) == file_id & minCutLog(:,6) == pc_no);
        %     mcidx = minCutLog(idx,1);
        mc_configs = minCutLog(idx,1:4);
        if (size(mc_configs,1) > 1)
        clusters = clusterdata(mc_configs(:,2:3),'criterion','distance','cutoff',0.7);
        [c_freq,c_id]=hist(clusters,unique(clusters));
        [~,max_c_idx] = max(c_freq);
        max_c_id = c_id(max_c_idx);
        cidx = find(clusters==max_c_id);
        
        safeMincut = mc_configs(cidx,:);
        safeMincutLog = [safeMincutLog; [[safeMincut file_id*ones(size(safeMincut,1),1) pc_no*ones(size(safeMincut,1),1)]]];
        
        %     valididx = find(configs(:,5) == 1);
        %     valid = configs(valididx,:);
        %     plot(configs(:,1),configs(:,2),'r.')
        
        %     plot(valid(:,1),valid(:,2),'g.');
        %     plot(mc_configs(:,2),mc_configs(:,3),'bs');
        %     hold on
        %     plot(safeMincut(:,2),safeMincut(:,3),'ks');
        %     axis([-4 4 -11 11])
        %     hold off
        safeMincutCount = [safeMincutCount; [size(safeMincut,1) file_id pc_no]];
        else
            safeMincutLog = [safeMincutLog; [[mc_configs file_id*ones(size(mc_configs,1),1) pc_no*ones(size(mc_configs,1),1)]]];
            safeMincutCount = [safeMincutCount; [size(mc_configs,1) file_id pc_no]];
        end
    end
end
% maxSafeValue = max(safeMincutCount(:,1));

save('logs/tmp','safeMincutCount','mincutCount','safeMincutLog','minCutLog','connCompsLog')

end

