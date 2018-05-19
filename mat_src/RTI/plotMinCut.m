function [ output_args ] = plotMinCut( input_args )
%PlotMinCut Plot min cut on PRM graph in xy plane
%   Detailed explanation goes here
close all

%% vehicle info
x_min = -3;
x_max = 3;
y_min = -10;
y_max = 10;
x_config_count = 25;
y_config_count = 41;
theta_config_count = 7;
costLog = [];
costMatLog = [];

start_n_goal_nodes_factore = 1;

%% load data from files
adjDirectory = '/home/az/git_repos/phd/road-traversability/data/adjacency/clearance2/vehicle2/clear';
configDirectory =  '/home/az/git_repos/phd/road-traversability/data/configurations/clearance2/vehicle2/clear';
file_id = 15;
adjFile = strcat(adjDirectory,int2str(file_id),'_adj');
configFile = strcat(configDirectory,int2str(file_id),'_conf');

Ds = importdata(adjFile,' ');
configs = importdata(configFile,' ');

% startNodes = find(configs(:,2) < y_min+2 & configs(:,3) == 90);
startNodes = find(configs(:,2) < y_min+start_n_goal_nodes_factore);
goalNodes = find(configs(:,2) > y_max-start_n_goal_nodes_factore);
total_config = size(configs,1);

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
[costMat minCutNodes]= PRMAnalysisSparse(A,startNodes,goalNodes,total_config);
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

%% load data from mat files
% load('road_rsi_quasi_01.mat');
% for i=1:100
%     close all
% frame_id = randi(25,1,1)
%
% % get frame configs
% idx = find(configsLog(:,7) == frame_id);
% configs = configsLog(idx,1:6);
%
% % get frame mincut nodes
% idx = find(minCutLog(:,2) == frame_id);
% mincutNodes = minCutLog(idx,1);
%
% % get frame safe mincut nodes
% idx = find(minCutLog(:,2) == frame_id);
% safeMincutNodes = minCutLog(idx,1);
%
% % get start nodes
% idx = find(startNodesLog(:,2) == frame_id);
% startNodes = startNodesLog(idx,1);
%
% % get goal nodes
% idx = find(goalNodesLog(:,2) == frame_id);
% goalNodes = goalNodesLog(idx,1);
%
% % get adjacency data
% idx = find(adjacencyLog(:,4) == frame_id);
% adjacency = adjacencyLog(idx,1:3);
%
% G = digraph(spconvert(adjacency));
%
% xCord = configs(:,1);
% yCord = configs(:,2);
% h = plot(G,'XData',xCord,'yData',yCord,'MarkerSize',3);
% hold on
% axis ([-3.5 3.5 -11 11])
% % h = plot(G,'XData',xCord,'yData',yCord,'NodeLabel',configs(:,3),'MarkerSize',3);
%
% % idx = find(configs(minCutNodes(:,1),3) == 90);
% % nodeId = minCutNodes(idx(randi(numel(idx),1,1)),1);
% % % % nidx =
% % % % sG = subgraph(G,nodeId);
% % sucIDs = successors(G,nodeId);
% % preIDs = predecessors(G,nodeId);
% % %
% % highlight(h,[nodeId],'NodeColor','k','MarkerSize',4)
% % highlight(h,[sucIDs' preIDs'],'NodeColor','g','MarkerSize',4)
%
% % highlight(h,[nodeId],'NodeColor','g','MarkerSize',4)
% % plot(G,'XData',xCord,'yData',yCord,'NodeLabel',configs(:,3),'MarkerSize',3);
% % hold on
% %
% valid_idx = find(configs(:,5)==1);
% valid = configs(valid_idx,:);
% invalid_idx = find(configs(:,5)==0);
% invalid = configs(invalid_idx,:);
%
% plot(valid(:,1),valid(:,2),'g.');
% plot(invalid(:,1),invalid(:,2),'r.');
%
% mc_x = configs(mincutNodes(:,1),1);
% mc_y = configs(mincutNodes(:,1),2);
% plot(mc_x,mc_y,'s','MarkerSize',7,'MarkerFaceColor','b');
%
% smc_x = configs(safeMincutNodes(:,1),1);
% smc_y = configs(safeMincutNodes(:,1),2);
% plot(smc_x,smc_y,'s','MarkerSize',7,'MarkerFaceColor','g');
%
% plot(configs(startNodes,1),configs(startNodes,2),'bs');
% plot(configs(goalNodes,1),configs(goalNodes,2),'ks');
%
% % mc_configs = configs(minCutNodes(:,1),:);
% % x_prm = mc_configs(:,1);
% % y_prm = mc_configs(:,2);
% % t_prm = mc_configs(:,3);
% % u_prm = cosd(t_prm);
% % v_prm = sind(t_prm);
% % quiver(x_prm,y_prm,u_prm,v_prm,0.5,'k')
%
% % legend('Configs Connection', 'Valid Configs', 'Invalid Configs', 'Mincut Configs', 'Start Configs', 'Goal Configs')
% xlabel('Width')
% ylabel('Length')
% pause(5)
% end
%%
% plot obstacle
% obs_directory = '/home/khan/phd_ws/traversability/pointclouds/long_roads/road_long';
% obsFile = strcat(obs_directory,int2str(file_id),'_obs');
% obs_file = importdata(obsFile,' ');
%
% hold on
% for i=1:size(obs_file,1)
%     r = obs_file(i,3);
%     d = r*2;
%     px = obs_file(i,1)-r;
%     py = obs_file(i,2)-r;
%     rectangle('Position',[px py d d],'Curvature',[1,1]);
% end
%%
% idx = find(configs(minCutNodes(:,1),3)==60);
% mc_x = configs(minCutNodes(idx,1),1);
% mc_y = configs(minCutNodes(idx,1),2);
% plot(mc_x,mc_y,'s','MarkerSize',7,'MarkerFaceColor','k');
%
% xlabel('Road width');
% ylabel('Road length');
% axis([-4 4 -11 11]);
%
% hold off
% figure
% valid = find(configs(:,5)==1);
% invalid = find(configs(:,5)==0);
% x_prm = configs(:,1);
% y_prm = configs(:,2);
% t_prm = configs(:,3);
% u_prm = cosd(t_prm);
% v_prm = sind(t_prm);
% quiver(x_prm(valid),y_prm(valid),u_prm(valid),v_prm(valid),0.5,'g')
% hold on
% quiver(x_prm(invalid),y_prm(invalid),u_prm(invalid),v_prm(invalid),0.5,'r')
% axis ([-3.5 3.5 -11 11])
%

%
% % clusters = clusterdata(mc_configs(:,1:2),'criterion','distance','cutoff',0.7);
% % [c_freq,c_id]=hist(clusters,unique(clusters))
% % plot(mc_configs(:,1),mc_configs(:,2),'b*');
% % hold on
% % [~,max_c_idx] = max(c_freq);
% % max_c_id = c_id(max_c_idx);
% % idx = find(clusters==max_c_id);
% % % plot(mc_configs(idx,1),mc_configs(idx,2),'r*');
% % c_configs = mc_configs(idx,:);
% % validIdx = find(configs(:,5)==1);
% % valid_configs = configs(validIdx,:);
% % [min_vals] = min(mc_configs(idx,1:2),[],1);
% % [max_vals] = max(mc_configs(idx,1:2),[],1);
% % safest_distance = max(max_vals - min_vals)
%
% figure
% histogram(mc_configs(:,3),theta_config_count)
% save('minccut_configs.mat','mc_configs','configs')
end

