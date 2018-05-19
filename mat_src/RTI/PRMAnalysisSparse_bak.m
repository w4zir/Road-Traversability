function [ output_args ] = PRMAnalysisRandom( input_args )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
% close all;

 wid = 25;
 len = 40;
 theta = 7;
roadWidth = 4;
roadStandardWidth = 4;
 
dim = wid*len*theta;
costLog = [];
costMatLog = [];

% dim = size(A);
    
 str = '/home/mudassir/phd_ws/data/traversability/adjacency/kitti/2011_09_26_drive_0087/v2/road_'
 startStr = '/home/mudassir/phd_ws/data/traversability/matlab_data/';
 fileNameStart =  strcat(startStr,'road_start_175');
 fileNameGoal =  strcat(startStr,'road_goal_175');
 startNodes = importdata(fileNameStart,'\n');
 startNodes = startNodes+1;
 goalNodes = importdata(fileNameGoal,'\n');
 goalNodes = goalNodes+1;

for j=0:145
%     dim=7000;
    j
    A = [];
    As = [];
    Ae = [];
%     j=indx(i)
%str = '~/phd_ws/data/adjacency/test/road_obs'
     fileNameAdj =  strcat(str,int2str(j),'_adj');
     
     if(j==35)
         a = 1
     end
%     fileNameStart =  strcat('~/phd_ws/data/adjacency/synthetic/v1/v1_rand_40x60/road_obs',int2str(j),'_start');
%     fileNameGoal =  strcat('~/phd_ws/data/adjacency/synthetic/v1/v1_rand_40x60/road_obs',int2str(j),'_goal');
%     fileNameAdj =  strcat('~/phd_ws/data/adjacency/synthetic/v1/v1_rand_40x60/road_obs',int2str(j),'_adj');
%      startNodes = importdata(fileNameStart,'\n');
     
      Ds = importdata(fileNameAdj,' ');
      % if no valid connection was found
      if(length(Ds)==0) 
          costLog = [costLog; 0];
          continue;
      end
      Ds(:,1:2) = Ds(:,1:2) +1;
      if(length(find(Ds(:,1)==dim))==0 || length(find(Ds(:,2)==dim))==0 )
          Ds = [Ds;[dim dim 0]];
      end
     
     % convert into sparse matrix
      As = spconvert(Ds);
      A = full(As);
%       indx = 4:7:dim;
%       Azero = A(indx,indx);
%       As = sparse(Azero);
%       A = Azero;
%       dim=1000;
%       startNodes = [1:25]';
%       goalNodes = [976:1000]';
    %     h = view(biograph(As,[],'ShowWeights','on'))
    
    [C S] = graphconncomp(As);
    sCC = S(startNodes);
    eCC = S(goalNodes);
    costMat = [];
    nonCC = [];
    for i=1:dim(1)
        nonCC(i) = length(find(S(1:i)~=S(i)));
    end
    for i=1:C
        if(length(find(sCC==i))==0 || length(find(eCC==i))==0 || length(find(S==i))==0)
            continue;
        end
        %Ai = A(find(S==i),find(S==i));
        ccNodes = find(S==i);
        newStartNodes = startNodes(find(sCC==i));%-nonCC(startNodes(find(sCC==i)));
        newGoalNodes = goalNodes(find(eCC==i));%-nonCC(goalNodes(find(eCC==i)));
        midNodes = setdiff([ccNodes],[[newStartNodes;newGoalNodes]]);
%         midNodes = setdiff([1:size(Ai,1)],[[newStartNodes] [newEndNodes]]);

        noMidNodes = length(midNodes);
        Ai = zeros(2*noMidNodes+2,2*noMidNodes+2);
        
        % set start to mid nodes connection
        s2Mid = max(A(newStartNodes,:),[],1);
        Ai(1,2:noMidNodes+1) = 1000*s2Mid(midNodes);
        
        % mid nodes to replicated mid nodes connections
        Ai(2:noMidNodes+1,noMidNodes+2:2*noMidNodes+1) = eye(noMidNodes);
        
        % replicated mid nodes to mid nodes connection
        Ai(noMidNodes+2:2*noMidNodes+1,2:noMidNodes+1) = 1000*A(midNodes,midNodes);
        
        % set mid nodes to goal nodes connection
        g2Mid = max(A(newGoalNodes,:),[],1);
        Ai(noMidNodes+2:2*noMidNodes+1,2*noMidNodes+2) = 1000*g2Mid(midNodes)';
        
        [flowval cut R H] = max_flow(sparse(Ai),1,2*noMidNodes+2);
        sCluster = find(cut==1);
        sCluster(find(sCluster==1))=[];
        gCluster = find(cut==-1);
        gCluster(find(gCluster==(2*noMidNodes+2)))=[];
        gClusterMod = gCluster-noMidNodes;
        remNodesIndx = intersect(sCluster,gClusterMod);
        remNodesIndx = remNodesIndx - 1; % subtract start index
        remNodes = midNodes(remNodesIndx);
        costMat = [costMat; length(newStartNodes) length(newGoalNodes) length(midNodes) flowval];
    end
    costMatLog = [costMatLog; costMat j*ones(size(costMat,1),1)];
    if(length(costMat)==0)
        cost = 0;
    else
        cost = sum(costMat(:,4));
    end
    costLog = [costLog; cost];    
 end
    maxVal = max(costLog);
    costLog = costLog./maxVal;

         save('2011_09_26_drive_0087_v2','costLog','costMatLog','maxVal','startNodes','goalNodes')
    
figure
plot(costLog);
% [mv, mw] = mincut(A, startNodes);
end

