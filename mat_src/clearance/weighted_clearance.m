close all
clear all
clc

%% vehicle info
x_min = -3;
x_max = 3;
y_min = -10;
y_max = 10;
x_config_count = 25;
y_config_count = 81;
theta_config_count = 7;
costLog = [];
costMatLog = [];

start_n_goal_nodes_factore = 1;

%% load data from files
adjDirectory = '/home/khan/git_repos/phd/road-traversability/data/adjacency/clearance2/vehicle2/clear';
configDirectory =  '/home/khan/git_repos/phd/road-traversability/data/configurations/clearance2/vehicle2/clear';
file_id = 9;
adjFile = strcat(adjDirectory,int2str(file_id),'_adj');
configFile = strcat(configDirectory,int2str(file_id),'_conf');

Ds = importdata(adjFile,' ');
configs = importdata(configFile,' ');

% startNodes = find(configs(:,2) < y_min+2 & configs(:,3) == 90);
startNodes = find(configs(:,2) < y_min+start_n_goal_nodes_factore);
goalNodes = find(configs(:,2) > y_max-start_n_goal_nodes_factore);
total_config = size(configs,1);
valid_nodes = find(configs(:,5) == 1);
invalid_nodes = find(configs(:,5) == 0)
node_weights = configs(valid_nodes,6);

Ds = Ds(:,1:3);
Ds(:,1:2) = Ds(:,1:2) +1;
adj_mat = Ds;
Ds(:,3) = configs(Ds(:,1),3);
Ds = [Ds; [Ds(:,2) Ds(:,1) Ds(:,3)]];
Ds = [Ds;[total_config total_config 0]];
adj_mat = [adj_mat;[total_config total_config 0]];

% % convert into sparse matrix and then graph
As = spconvert(Ds);
A = full(As);
degree = sum(A,2);
adj_mat(:,3) = degree(adj_mat(:,1));

G = digraph(spconvert(adj_mat));
G = graph(As);


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