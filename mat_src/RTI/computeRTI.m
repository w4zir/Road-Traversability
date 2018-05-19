function [ output_args ] = computeRTI( input_args )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% directory = '/home/mudassir/phd_ws/data/traversability/adjacency/kitti/2011_09_26_drive_0087/v2/road_'
% start_directory = '/home/mudassir/phd_ws/data/traversability/matlab_data/';
% fileNameStart =  strcat(start_directory,'road_start_175');
% fileNameGoal =  strcat(start_directory,'road_goal_175');
%
%  startNodes = importdata(fileNameStart,'\n');
%  startNodes = startNodes+1;
%  goalNodes = importdata(fileNameGoal,'\n');
%  goalNodes = goalNodes+1;

x_config_cout = 17;
y_config_cout = 25;
theta_config_count = 5;
costLog = [];
costMatLog = [];

total_config = x_config_cout * y_config_cout * theta_config_count;

startNodes = [1:x_config_cout*theta_config_count]';
goalNodes = [total_config - x_config_cout*theta_config_count + 1:1:total_config]';


fileNameAdj = 'road_obs42_adj';  % strcat(str,int2str(j),'_adj');
Ds = importdata(fileNameAdj,' ');
Ds(:,1:2) = Ds(:,1:2) +1;
Ds = [Ds; [Ds(:,2) Ds(:,1) Ds(:,3)]];
Ds = [Ds;[total_config total_config 0]];

[costMat]= PRMAnalysisSparse(Ds,startNodes,goalNodes,x_config_cout,y_config_cout,theta_config_count);


maxVal = max(costLog);
costLog = costLog./maxVal;

costMatLog = [costMatLog; costMat j*ones(size(costMat,1),1)];
if(length(costMat)==0)
    cost = 0;
else
    cost = sum(costMat(:,4));
end
% save('test','costLog','costMatLog','maxVal','startNodes','goalNodes')

figure
plot(costLog);
% [mv, mw] = mincut(A, startNodes);

% if(length(Ds)==0)
%     costLog = [costLog; 0];
%     continue;
% end
%   [] = PRMAnalysisSparse();

end

