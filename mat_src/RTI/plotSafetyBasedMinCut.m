function [ output_args ] = plotSafetyBasedMinCut( input_args )
%PlotMinCut Plot min cut on PRM graph in xy plane
%   Detailed explanation goes here
close all

adjDirectory = '/home/khan/phd_ws/traversability/adjacency/rsi/tmp/road_rsi';
configDirectory =  '/home/khan/phd_ws/traversability/configs/rsi/tmp/road_rsi';

file_id = 5;

x_min = -3;
x_max = 3;
y_min = -10;
y_max = 10;
x_config_count = 25;
y_config_count = 81;
theta_config_count = 7;
costLog = [];
costMatLog = [];

% total_config = x_config_count * y_config_count * theta_config_count;
% 
% startNodes = [1:x_config_count*theta_config_count]';
% goalNodes = [total_config - x_config_count*theta_config_count + 1:1:total_config]';

% if(file_id < 10)
%     adjFile = strcat(directory,'0',int2str(file_id),'_adj');
% else
    adjFile = strcat(adjDirectory,int2str(file_id),'_adj');
    configFile = strcat(configDirectory,int2str(file_id),'_conf');
% end
% prmFile = strcat(prmDirectory,int2str(file_id),'_prm');

Ds = importdata(adjFile,' ');
configs = importdata(configFile,' ');

startNodes = find(configs(:,2) < y_min+0.5);
goalNodes = find(configs(:,2) > y_max-0.5);
total_config = size(configs,1);

Ds(:,1:2) = Ds(:,1:2) +1;
adj_mat = Ds;
% Ds(:,3) = configs(Ds(:,1),3);
Ds = [Ds; [Ds(:,2) Ds(:,1) Ds(:,3)]];
Ds = [Ds;[total_config total_config 0]];
adj_mat = [adj_mat;[total_config total_config 0]];

% convert into sparse matrix and then graph
As = spconvert(Ds);
A = full(As);
G = digraph(spconvert(adj_mat));

% xRes = (max_x - min_x)/(x_config_count-1);
% yRes = (max_y - min_y)/(y_config_count-1);
% xC = repmat(min_x:xRes:max_x,1,y_config_count);
% yC = reshape(repmat(min_y:yRes:max_y,x_config_count,1),1,length(xC));
% plot(G,'XData',xC,'yData',yC);

% PRM = importdata(prmFile,' ');
% return;
configs_weight = round(100*configs(:,6));
[costMat minCutNodes]= PRMAnalysisWithSafety(A,configs_weight,startNodes,goalNodes,total_config);
if(length(costMat)==0)
    cost = 0;
else
    cost = sum(costMat(:,4));
end
costLog = [costLog; cost];
minCutPatch = cost

xCord = configs(:,1);
yCord = configs(:,2);
h = plot(G,'XData',xCord,'yData',yCord,'MarkerSize',3);
% h = plot(G,'XData',xCord,'yData',yCord,'NodeLabel',configs(:,3),'MarkerSize',3);
hold on

% idx = find(configs(minCutNodes(:,1),3) == 90);
% nodeId = minCutNodes(idx(randi(numel(idx),1,1)),1);
% % % nidx = 
% % % sG = subgraph(G,nodeId);
% sucIDs = successors(G,nodeId);
% preIDs = predecessors(G,nodeId);
% % 
% highlight(h,[nodeId],'NodeColor','k','MarkerSize',4)
% highlight(h,[sucIDs' preIDs'],'NodeColor','g','MarkerSize',4)

% highlight(h,[nodeId],'NodeColor','g','MarkerSize',4)
% plot(G,'XData',xCord,'yData',yCord,'NodeLabel',configs(:,3),'MarkerSize',3);
% hold on
% 
valid_idx = find(configs(:,5)==1);
valid = configs(valid_idx,:);
invalid_idx = find(configs(:,5)==0);
invalid = configs(invalid_idx,:);
 
mc_x = configs(minCutNodes(:,1),1);
mc_y = configs(minCutNodes(:,1),2);

plot(valid(:,1),valid(:,2),'g.');
plot(invalid(:,1),invalid(:,2),'r.');
plot(mc_x,mc_y,'s','MarkerSize',7,'MarkerFaceColor','b');
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
% 
mc_configs = configs(minCutNodes(:,1),:)
save('minccut_configs.mat','mc_configs','configs')
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
figure
histogram(mc_configs(:,3),theta_config_count)

end

