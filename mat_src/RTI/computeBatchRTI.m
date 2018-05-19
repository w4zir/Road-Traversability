function [ output_args ] = computeBatchRTI( input_args )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% directory = '/home/khan/phd_ws/traversability/adjacency/narrow_passage/narrow_passage'
directory = '/home/khan/phd_ws/traversability/adjacency/rti_quasi/road_long'
% start_directory = '/home/mudassir/phd_ws/data/traversability/matlab_data/';
% fileNameStart =  strcat(start_directory,'road_start_175');
% fileNameGoal =  strcat(start_directory,'road_goal_175');
%
%  startNodes = importdata(fileNameStart,'\n');
%  startNodes = startNodes+1;
%  goalNodes = importdata(fileNameGoal,'\n');
%  goalNodes = goalNodes+1;

x_config_cout = 25;
y_config_cout = 41;
theta_config_count = 7;
costLog = [];
costMatLog = [];
minCutLog = [];

total_config = x_config_cout * y_config_cout * theta_config_count;

startNodes = [1:x_config_cout*theta_config_count]';
goalNodes = [total_config - x_config_cout*theta_config_count + 1:1:total_config]';

fileNameAdj = [];
for file_idx=0:23
    file_idx
%     if (file_idx<10)
%         fileNameAdj = strcat(directory,'0',int2str(file_idx),'_adj');
%     else
        fileNameAdj = strcat(directory,int2str(file_idx),'_5_adj');
%     end
    Ds = importdata(fileNameAdj,' ');
    if(length(Ds)==0)
        costLog = [costLog; 0];
        continue;
    end
    Ds(:,1:2) = Ds(:,1:2) +1;
    Ds = [Ds; [Ds(:,2) Ds(:,1) Ds(:,3)]];
    Ds = [Ds;[total_config total_config 0]];
    As = spconvert(Ds);
    A = full(As);    
    
    [costMat, minCutNodes]= PRMAnalysisSparse(A,startNodes,goalNodes,total_config);
    minCutLog = [minCutLog; [minCutNodes file_idx*ones(size(minCutNodes,1),1)]];
    if(file_idx > 48)
        
    end 
    costMatLog = [costMatLog; costMat file_idx*ones(size(costMat,1),1)];
    if(length(costMat)==0)
        cost = 0;
    else
        cost = sum(costMat(:,4));
    end
    costLog = [costLog; cost];
end

maxVal = max(costLog);
% costLog = costLog./maxVal;

save('logs/rti_quasi','costLog','minCutLog','costMatLog','maxVal','startNodes','goalNodes')

figure
bar(costLog);

end

