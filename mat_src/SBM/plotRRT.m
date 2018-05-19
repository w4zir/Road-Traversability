function [ output_args ] = plotRRT( input_args )
%UNTITLED Plot safe RRT graph
%   Detailed explanation goes here

close all
obs_directory = '/home/khan/phd_ws/traversability/pointclouds/long_roads/road_long';
adj_directory = '/home/khan/phd_ws/traversability/adjacency/rti_compare/road_long';
rrt_directory = '/home/khan/phd_ws/traversability/configs/rti_compare/road_long';

file_id = 0;

% if(file_id < 10)
%     obsFile = strcat(obs_directory,'0',int2str(file_id),'_obs');
%     adjFile = strcat(adj_directory,'0',int2str(file_id),'_adj');
%     rrtFile = strcat(rrt_directory,'0',int2str(file_id),'_rrt');
% else
% obsFile = strcat(obs_directory,int2str(file_id),'_1_obs');
adjFile = strcat(adj_directory,int2str(file_id),'_1_adj');
rrtFile = strcat(rrt_directory,int2str(file_id),'_1_prt');
% end

% Read obstacle, adjacency and configuration files
% obs_file = importdata(obsFile,' ');
adj_file = importdata(adjFile,' ');
configs = importdata(rrtFile,' ');

adj_file = adj_file + 1;

% create adjacency list
config_count = size(configs,1);
adj_list = [adj_file; [config_count config_count 0]];

% compute edge weights
% edge_configs = [configs(adj_list(:,1),1:2) configs(adj_list(:,2),1:2)];
% edge_weights = sqrt(sum((edge_configs(:,1)-edge_configs(:,3)).^2 + (edge_configs(:,2)-edge_configs(:,4)).^2,2));

edge_configs = [configs(adj_list(:,1),1:3) configs(adj_list(:,2),1:3)];
% get the graph
% As_adj = spconvert(adj_list,edge_weights);
% G = digraph(adj_list(:,1),adj_list(:,2), edge_weights);
G = digraph(adj_list(:,1),adj_list(:,2));

p = plot(G,'xData',configs(:,1),'yData',configs(:,2));

axis([-3 3 -12 12]);

% hold on
% for i=1:size(obs_file,1)
%     r = obs_file(i,3);
%     d = r*2;
%     px = obs_file(i,1)-r;
%     py = obs_file(i,2)-r;
%     rectangle('Position',[px py d d],'Curvature',[1,1]);
% end
% 
% % plot invalid colors with red
% invalid_idx = find(configs(:,5)==0);
% invalid_nodes = configs(invalid_idx,1:2);
% x = invalid_nodes(:,1);
% y = invalid_nodes(:,2);
% plot(x,y,'.','MarkerFaceColor','r');
% 
% % find path from start to goal
% start_idx = find(configs(:,2) < - 9.8);
% start_nodes = configs(start_idx,1:2);
% goal_idx = find(configs(:,2) > 9.8);
% goal_nodes = configs(goal_idx,1:2);
% 
% dist = distances(G,start_idx,goal_idx);
% [min_col,min_row_idx] = min(dist); 
% [min_val,min_col_idx] = min(min_col); 
% path = shortestpath(G,start_idx(min_row_idx(min_col_idx)),goal_idx(min_col_idx));
% 
% % path = [];
% % while(numel(path) <= 0)
% %     rand_start = start_idx(randi(numel(start_idx)));
% %     rand_goal = goal_idx(randi(numel(goal_idx)));
% %     
% %     path = shortestpath(G,rand_start,rand_goal);
% % end
% x_path = configs(path,1);
% y_path = configs(path,2);
% plot(x_path,y_path,'+-','LineWidth',3,'MarkerSize',4,'MarkerFaceColor','g');

% highlight(p,path,'EdgeColor','r')
end
