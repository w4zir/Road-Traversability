function [ costMat, minCutNodes, cc_count] = PRMAnalysisSparseWeighted( A,startNodes,goalNodes,total_config,weights,valid_nodes,invalid_nodes)
%Find mincut of wighted PRM graph
%   Detailed explanation goes here
% close all;

% very large connection value to remove edges from maxflow calculations
K = 100000;
% A = [];
% As = [];
% Ae = [];

% convert into sparse matrix and then graph
% As = spconvert(Ds);
% A = full(As);
% G = graph(As);
G = digraph(A);

% get weights of mid-to-mid nodes connection
mid2mid_weights = ones(total_config,1);
% weights =  weights/min(weights);
% weights =  10*weights/max(weights)
mid2mid_weights(valid_nodes) = weights;
% mid2mid_weights(valid_nodes) = 10*weights/max(weights);
fprintf(strcat('min weight: ',num2str(min(weights)),'\n'))
fprintf(strcat('max weight: ',num2str(max(weights)),'\n'))

[S] = conncomp(G);
C = max(S);
sCC = S(startNodes);
eCC = S(goalNodes);
costMat = [];
minCutNodes = [];
cc_count = 0;
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
    Ai(1,2:noMidNodes+1) = K*s2Mid(midNodes);
    
    % mid nodes to replicated mid nodes connections
    Ai(2:noMidNodes+1,noMidNodes+2:2*noMidNodes+1) = eye(noMidNodes).*repmat(mid2mid_weights(midNodes),1,noMidNodes);
%         Ai(2:noMidNodes+1,noMidNodes+2:2*noMidNodes+1) = eye(noMidNodes);
    
    % replicated mid nodes to mid nodes connection
    Ai(noMidNodes+2:2*noMidNodes+1,2:noMidNodes+1) = K*A(midNodes,midNodes);
    
    % set mid nodes to goal nodes connection
    g2Mid = max(A(newGoalNodes,:),[],1);
    Ai(noMidNodes+2:2*noMidNodes+1,2*noMidNodes+2) = K*g2Mid(midNodes)';
    
    %     % visualization code
    %     figure
    %     GG = digraph(Ai);
    %     plot(GG,'EdgeLabel',GG.Edges.Weight)
    
    
    % maxflow
    G = digraph(Ai);
%         [flowval cut R H] = max_flow(sparse(Ai),1,2*noMidNodes+2);
    [flowval cut cs ct] = maxflow(G,1,2*noMidNodes+2,'augmentpath');
%         [flowval1 cut R H] = maxflow(digraph(Ai),1,2*noMidNodes+2);
    
%     figure
%     H = plot(G,'Layout','layered','Sources',cs,'Sinks',ct, ...
%         'EdgeLabel',G.Edges.Weight);
%     highlight(H,cs,'NodeColor','red')
%     highlight(H,ct,'NodeColor','green')
%     
%     
%     figure
%     plot(cut,'EdgeLabel',cut.Edges.Weight)
    
    % find mincut nodes
    cs_ct_diff = cut.Edges.EndNodes(:,2) - cut.Edges.EndNodes(:,1);
    idx = find(cs_ct_diff == noMidNodes);
    possible_mincut_nodes = cut.Edges.EndNodes(idx,:)
    [~,ai,~] = intersect(possible_mincut_nodes(:,1),cs);
    [~,bi,~] = intersect(possible_mincut_nodes(:,2),ct);
    [~,mincut_idx,~] = intersect(ai,bi);
    cc_mincut_nodes = possible_mincut_nodes(ai(mincut_idx),1);
    mincut_nodes = midNodes(cc_mincut_nodes-1);
    
%         sCluster = find(cut==1);
%         sCluster(find(sCluster==1))=[];
%         gCluster = find(cut==-1);
%         gCluster(find(gCluster==(2*noMidNodes+2)))=[];
%         gClusterMod = gCluster-noMidNodes;
%         remNodesIndx = intersect(sCluster,gClusterMod);
%         remNodesIndx = remNodesIndx - 1; % subtract start index
%         remNodes = midNodes(remNodesIndx);
    cc_count = cc_count + 1;
    minCutNodes = [minCutNodes; [mincut_nodes' cc_count*ones(size(mincut_nodes))']];
    costMat = [costMat; length(newStartNodes) length(newGoalNodes) length(midNodes) flowval];    
end
end

