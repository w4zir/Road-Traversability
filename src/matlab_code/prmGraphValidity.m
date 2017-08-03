function [ output_args ] = prmGraphValidity( input_args )
%CHeck whether the graph generated obeys the constrains imposed
%   Detailed explanation goes here

% close all

adjDirectory = '/home/khan/phd_ws/traversability/adjacency/long_roads_quasi2/road_long';
configDirectory =  '/home/khan/phd_ws/traversability/configs/long_roads_quasi2/road_long';

file_id = 23;

adjFile = strcat(adjDirectory,int2str(file_id),'_adj');
configFile = strcat(configDirectory,int2str(file_id),'_prt');

Ds = importdata(adjFile,' ');
configs = importdata(configFile,' ');
Ds(:,1:2) = Ds(:,1:2) +1;

nc = [configs(Ds(:,1),1:3) configs(Ds(:,2),1:3)];
angle_diff = nc(:,3)-nc(:,6);
angle_diff = abs(angle_diff);

log = [];

for diff = 0:10:120
    idx = find(angle_diff > diff);
    log = [log; size(idx,1)];
end
log

valid_idx = find(configs(:,5)==1);
valid = configs(valid_idx,:);
invalid_idx = find(configs(:,5)==0);
invalid = configs(invalid_idx,:);

invalid_in_adj = ismember(invalid_idx,Ds(:,1:2));
idx = find(invalid_in_adj==1);
size(idx)



end