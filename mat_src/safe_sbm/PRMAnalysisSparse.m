function [ costMat, minCutNodes] = PRMAnalysisRandom( A,startNodes,goalNodes,total_config)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
% close all;

% A = [];
% As = [];
% Ae = [];

% convert into sparse matrix and then graph
% As = spconvert(Ds);
% A = full(As);
% G = graph(As);
G = graph(A);

[S] = conncomp(G);
C = max(S);
sCC = S(startNodes);
eCC = S(goalNodes);
costMat = [];
minCutNodes = [];
minCutCount = 1;
nonCC = [];
for i=1:total_config
    nonCC(i) = length(find(S(1:i)~=S(i)));
end
for i=1:C
    if(length(find(sCC==i))==0 || length(find(eCC==i))==0 || length(find(S==i))==0)
        continue;
    end
    
    ccNodes = find(S==i);
    newStartNodes = startNodes(find(sCC==i));%-nonCC(startNodes(find(sCC==i)));
    newGoalNodes = goalNodes(find(eCC==i));%-nonCC(goalNodes(find(eCC==i)));
    midNodes = setdiff([ccNodes],[[newStartNodes;newGoalNodes]]);
    
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
%     [flowval1 cut1 R1 H1] = maxflow(digraph(Ai),1,2*noMidNodes+2);
    sCluster = find(cut==1);
    sCluster(find(sCluster==1))=[];
    gCluster = find(cut==-1);
    gCluster(find(gCluster==(2*noMidNodes+2)))=[];
    gClusterMod = gCluster-noMidNodes;
    remNodesIndx = intersect(sCluster,gClusterMod);
    remNodesIndx = remNodesIndx - 1; % subtract start index
    remNodes = midNodes(remNodesIndx);
    minCutNodes = [minCutNodes; [remNodes' minCutCount*ones(size(remNodes))']];
    costMat = [costMat; length(newStartNodes) length(newGoalNodes) length(midNodes) flowval];
end
end

