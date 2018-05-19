function [ output_args ] = plotConfigs( input_args )
%Plot configurations
%   Detailed explanation goes here

directory = '/home/khan/phd_ws/traversability/configs/long_roads_quasi/1_0/road_long'

x_min = -3;
x_max = 3;
y_min = -10;
y_max = 10;
theta_config_count = 13;

costLog = [];
costMatLog = [];

fildId = 1;


fileNameAdj = strcat(directory,int2str(fildId),'_conf');
configs = importdata(fileNameAdj,' ');

startNodes = configs(find(configs(:,2) < y_min+0.5),1:3);
goalNodes = configs(find(configs(:,2) > y_max-0.5),1:3);

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


plot(startNodes(:,1),startNodes(:,2),'b*');
plot(goalNodes(:,1),goalNodes(:,2),'g*');

end

