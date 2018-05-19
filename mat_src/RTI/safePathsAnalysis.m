function [ output_args ] = safePathsAnalysis( input_args )
%PathsAnalysis Cost analysis of safe paths.
%   Detailed explanation goes here
close all
adjDirectory = '/home/khan/phd_ws/traversability/adjacency/long_roads_quasi/road_long';
configDirectory =  '/home/khan/phd_ws/traversability/configs/long_roads_quasi/road_long';
% prmDirectory =  '/home/khan/phd_ws/traversability/prm/synthetic/road_obs';

file_id = 2;

delta = 1;

min_x = -3;
max_x = 3;
min_y = -10;
max_y = 10;
min_t = 60;
max_t = 120;
x_config_count = 25;
y_config_count = 81;
theta_config_count = 7;

total_config = x_config_count * y_config_count * theta_config_count;

startNodes = [1:x_config_count*theta_config_count]';
goalNodes = [total_config - x_config_count*theta_config_count + 1:1:total_config]';

% if(file_id < 10)
%     adjFile = strcat(directory,'0',int2str(file_id),'_adj');
%     prmFile = strcat(prm_directory,'0',int2str(file_id),'_prm');
% else
    adjFile = strcat(adjDirectory,int2str(file_id),'_adj');
    configFile = strcat(configDirectory,int2str(file_id),'_prt');
% end
% prmFile = strcat(prmDirectory,int2str(file_id),'_prm');

% Read prm and configuration files
prm_adj_list = importdata(adjFile,' ');
configs = importdata(configFile,' ');

startNodes = find(configs(:,2) < min_y+0.5);
goalNodes = find(configs(:,2) > max_y-0.5);
total_config = size(configs,1);

[ path_length_log, path_safety_log, paths_log, x_nodes, y_nodes] = safeQuasiPathService(prm_adj_list, configs, min_x, max_x, ...
    min_y, max_y, min_t, max_t, theta_config_count);

safety_per_unit_dist = path_safety_log./path_length_log;

plot(delta, path_length_log, 'b-o')
hold on
plot(delta, path_safety_log, 'r-+')
plot(delta, safety_per_unit_dist, 'g-*')

xlabel('delta')
ylabel('distance')
legend('path length', 'path safety')

figure
valid = find(configs(:,5)==1);
invalid = find(configs(:,5)==0);

x_prm = configs(:,1);
y_prm = configs(:,2);
t_prm = configs(:,3);
u_prm = cosd(t_prm);
v_prm = sind(t_prm);
quiver(x_prm(valid),y_prm(valid),u_prm(valid),v_prm(valid),0.5,'g')
hold on
quiver(x_prm(invalid),y_prm(invalid),u_prm(invalid),v_prm(invalid),0.5,'r')

x_coord = min_x:0.25:max_x;
y_coord = min_y:0.25:max_y;
colors = ['r';'k';'b'];
marker_types = ['-o';'-+';':*']
delta_to_plot = [0 0.5 1];
for i=1:numel(delta_to_plot)
    path_idx = find(paths_log(:,2)==delta_to_plot(i));
    path = paths_log(path_idx,1);
    x_path = x_coord(x_nodes(path));
    y_path = y_coord(y_nodes(path));
    plot(x_path,y_path,marker_types(i,:),'LineWidth',1,'MarkerSize',3,'MarkerFaceColor',colors(i));
end

obs_directory = '/home/khan/phd_ws/traversability/pointclouds/long_roads/road_long';
obsFile = strcat(obs_directory,int2str(file_id),'_obs');
obs_file = importdata(obsFile,' ');

hold on
for i=1:size(obs_file,1)
    r = obs_file(i,3);
    d = r*2;
    px = obs_file(i,1)-r;
    py = obs_file(i,2)-r;
    rectangle('Position',[px py d d],'Curvature',[1,1]);
end
end
