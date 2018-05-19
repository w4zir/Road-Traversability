function [ output_args ] = plotConnectingNeighbors( input_args )
%Connecting two configurations using kinematic constraints
%   Detailed explanation goes here
close all
hold on

adjDirectory = '/home/khan/phd_ws/traversability/adjacency/long_roads_quasi/tmp/road_long';
configDirectory =  '/home/khan/phd_ws/traversability/configs/long_roads_quasi/tmp/road_long';

file_id = 1;

x_min = -3;
x_max = 3;
y_min = -10;
y_max = 10;
x_config_count = 25;
y_config_count = 81;
theta_config_count = 7;
costLog = [];
costMatLog = [];

L = 2;
v = 1;
beta = 30;
alpha = 40;
max_alpha = 40;
n_radius = 2;

% p0 = [1;0;90];
p1 = [0   0   90];

% radius of circular motion
R = L/tand(max_alpha);

% centre of circles
cx1 = p1(1) - sind(p1(3))*R;
cy1 = p1(2) + cosd(p1(3))*R;
cx2 = p1(1) + sind(p1(3))*R;
cy2 = p1(2) - cosd(p1(3))*R;

plot(p1(1), p1(2),'k*');
hold on
plot(cx1, cy1,'b*');
plot(cx2, cy2,'b*');
rectangle('Position',[cx1-R cy1-R 2*R 2*R],'Curvature',[1 1]);
rectangle('Position',[cx2-R cy2-R 2*R 2*R],'Curvature',[1 1]);
rectangle('Position',[p1(1)-n_radius p1(2)-n_radius 2*n_radius 2*n_radius],'Curvature',[1 1]);
% plot(0, 0.5,'b*');

%% plot neighbors
adjFile = strcat(adjDirectory,int2str(file_id),'_adj');
configFile = strcat(configDirectory,int2str(file_id),'_conf');

adj_mat = importdata(adjFile,' ');
configs = importdata(configFile,' ');

startNodes = find(configs(:,2) < y_min+0.5);
goalNodes = find(configs(:,2) > y_max-0.5);
total_config = size(configs,1);

adj_mat(:,1:2) = adj_mat(:,1:2) +1;

id = find(configs(:,1) == -2 & configs(:,2) == 0 & configs(:,3) == 90);
nn_idx = find(adj_mat(:,1) == id);
n_idx = adj_mat(nn_idx,2);

n_configs = configs(n_idx,:)
x_prm = n_configs(:,1);
y_prm = n_configs(:,2);
t_prm = n_configs(:,3);
u_prm = cosd(t_prm);
v_prm = sind(t_prm);
quiver(x_prm,y_prm,u_prm,v_prm,0.5,'b');

end


