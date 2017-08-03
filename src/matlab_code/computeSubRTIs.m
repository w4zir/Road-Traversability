function [ output_args ] = computeSubRTIs( input_args )
%Sub-RTI: compute RTI of portions of the road patch
%   Road patch is divided into blocks of length 2m and RTI is calculated
%   for each block and compared to the total RTI of the patch.

directory = '/home/khan/phd_ws/traversability/adjacency/test/road_obs'
file_id = 45;

x_config_count = 17;
y_config_count = 25;
theta_config_count = 1;
costLog = [];
costMatLog = [];

total_config = x_config_count * y_config_count * theta_config_count;

startNodes = [1:x_config_count*theta_config_count]';
goalNodes = [total_config - x_config_count*theta_config_count + 1:1:total_config]';

if(file_id < 10)
    adjFile = strcat(directory,'0',int2str(file_id),'_adj');
else
    adjFile = strcat(directory,int2str(file_id),'_adj');
end

Ds = importdata(adjFile,' ');

Ds(:,1:2) = Ds(:,1:2) +1;
Ds = [Ds; [Ds(:,2) Ds(:,1) Ds(:,3)]];
Ds = [Ds;[total_config total_config 0]];
% convert into sparse matrix and then graph
As = spconvert(Ds);
A = full(As);
G = graph(As);

[costMat minCutNodes]= PRMAnalysisSparse(A,startNodes,goalNodes,total_config);
if(length(costMat)==0)
    cost = 0;
else
    cost = max(costMat(:,4));
end
costLog = [costLog; cost];
minCutPatch = cost

maxVal = max(costLog);
costLog = costLog./maxVal;
costMatLog = [];
costLog = [];
%----------------------------------------------
nodes = [1:x_config_count*y_config_count]';
nodes_x = [1:x_config_count];
nodes_y = [1:y_config_count];
[xCord yCord] = meshgrid(nodes_x,nodes_y);

xCord = reshape(xCord',1,x_config_count*y_config_count);
yCord = reshape(yCord',1,x_config_count*y_config_count);
plot(G,'XData',xCord,'yData',yCord);
%----------------------------------------------
x_config_count = 17;
% theta_config_count = 5;
y_block_length = 3;
yInc = 1;
for j=1:yInc:y_config_count-y_block_length
%     j    
    
    minConfig = (j-1)*x_config_count*theta_config_count+1;
    maxConfig = ((j-1) + y_block_length)*x_config_count*theta_config_count;
    
    total_config = x_config_count * (y_block_length) * theta_config_count;
    
    startNodes = [1:x_config_count*theta_config_count]';
    goalNodes = [total_config - x_config_count*theta_config_count + 1:1:total_config]';
    %     startNodes = [minConfig:minConfig+x_config_cout*theta_config_count-1]';
    %     startNodes = startNodes - minConfig + 1;
    %     goalNodes = [total_config - x_config_cout*theta_config_count + 1:1:total_config]';
    %     goalNodes = goalNodes - minConfig + 1;
    
    Aj = A(minConfig:maxConfig,minConfig:maxConfig);
    %     [ix, iy] = find(Ds>=minConfig & Ds<=maxConfig);
    %     idx = unique(ix);
    %     Dj = (Ds(idx,:));
    %     Ds = importdata(fileNameAdj,' ');
    %
    %     Ds(:,1:2) = Ds(:,1:2) +1;
    %     Ds = [Ds; [Ds(:,2) Ds(:,1) Ds(:,3)]];
    %     Ds = [Ds;[total_config total_config 0]];
    
    [costMat minCutNodes]= PRMAnalysisSparse(Aj,startNodes,goalNodes,total_config);
    if(length(costMat)==0)
        cost = 0;
    else
        cost = max(costMat(:,4));
    end
    costLog = [costLog; cost];
    costMatLog = [costMatLog; costMat j*ones(size(costMat,1),1)];
    %--------------------------------------------------
    mincut_nodes = minConfig+ceil(minCutNodes(:,1)/theta_config_count)-1;
    mc_x = mod(mincut_nodes,x_config_count);
    mc_y = ceil(mincut_nodes/x_config_count);

    hold on
    plot(mc_x,mc_y,'r*');
    %--------------------------------------------------
end
% costMatLog
costLog'
% minCutSubPatches = min(costLog)

end

