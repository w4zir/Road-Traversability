close all
clear all
clc

load('/home/az/git_repos/phd/road-traversability/mat_src/clear4.mat')
% L = laplacian(G);
% [eVal,eVec] = eig(full(L));
As = G.adjacency;
A = full(As);
% startNodes = 379
% goalNodes = 428
start_n_goal_nodes_factore = 0.5;
startNodes = find(configs(:,2) < y_min+start_n_goal_nodes_factore);
goalNodes = find(configs(:,2) > y_max-start_n_goal_nodes_factore);

total_config = config_count;
% % compute mincut
[costMat minCutNodes]= PRMAnalysisSparseWeighted(A,startNodes,goalNodes,total_config,node_weights,valid_nodes,invalid_nodes);
% [costMat minCutNodes]= PRMAnalysisSparse(A,startNodes,goalNodes,total_config);
size(minCutNodes,1)

xC = configs(:,1);
yC = configs(:,2);
plot(G,'XData',xC,'yData',yC);
hold on
valid_idx = find(configs(:,5)==1);
valid = configs(valid_idx,:);
invalid_idx = find(configs(:,5)==0);
invalid = configs(invalid_idx,:);
plot(valid(:,1),valid(:,2),'g.');
plot(invalid(:,1),invalid(:,2),'r.');

mc_x = configs(minCutNodes(:,1),1);
mc_y = configs(minCutNodes(:,1),2);
plot(mc_x,mc_y,'s','MarkerSize',7,'MarkerFaceColor','g');

plot(configs(startNodes,1),configs(startNodes,2),'bs');
plot(configs(goalNodes,1),configs(goalNodes,2),'ks');

legend('Configs Connection', 'Valid Configs', 'Invalid Configs', 'Mincut Configs', 'Start Configs', 'Goal Configs')
xlabel('Width')
ylabel('Length')


% calculate and plot safemincut
mc_configs = configs(minCutNodes(:,1),:);
clusters = clusterdata(mc_configs(:,1:2),'criterion','distance','cutoff',0.4);
[c_freq,c_id]=hist(clusters,unique(clusters));
[~,max_c_idx] = max(c_freq);
max_c_id = c_id(max_c_idx);
cidx = find(clusters==max_c_id);
smc_idx = minCutNodes(cidx,1);
safeMincut = configs(smc_idx,:);

smc_x = safeMincut(:,1);
smc_y = safeMincut(:,2);
plot(smc_x,smc_y,'s','MarkerSize',7,'MarkerEdgeColor','k','MarkerFaceColor','g');