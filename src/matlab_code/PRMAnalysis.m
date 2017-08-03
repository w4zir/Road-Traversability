function [ output_args ] = PRMAnalysis( input_args )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
close all;

% n = 8;
% m = 3;
type = 1;
%  A = importdata('data\road_cloud.pcd_adj',' ');
%  [xSize ySize] = size(A);
 n = 40;
 m = 60;
% isDisconnected = 1;
startNodes = [1:m];
endNodes = [m*n-m+1:1:m*n];
% startNodes = [1];
% endNodes = [7];
V = n*m; % total no of vertices
Ns = length(startNodes); % no of start nodes
Ne = length(endNodes); % no of start nodes
X = min(n,m); % no of start nodes
costLog = [];
costMatLog = [];
% indx = [0;22;23];
for j=1:1
    j
%     j=indx(i)
    fileName =  strcat('~/phd_ws/data/adjacency/synthetic/v1/v1_rand_40x60/road_obs',int2str(j),'_adj');
     A = importdata(fileName,' ');
     A(find(A==-1))=0;
%     [A D] = generateGridAdjacency(n,m,type);
%     plot_adj_mat(A,n,m,j);
    As = sparse(A);
    %     h = view(biograph(As,[],'ShowWeights','on'))
    [C S] = graphconncomp(As);
    sCC = S(startNodes);
    eCC = S(endNodes);
    costMat = [];
    nonCC = [];
    for i=1:size(A,1)
        nonCC(i) = length(find(S(1:i)~=S(i)));
    end
    for i=1:C
        if(length(find(sCC==i))==0 || length(find(eCC==i))==0 || length(find(S==i))==0)
            continue;
        end
        Ai = A(find(S==i),find(S==i));
        newStartNodes = startNodes(find(sCC==i))-nonCC(startNodes(find(sCC==i)));
        newEndNodes = endNodes(find(eCC==i))-nonCC(endNodes(find(eCC==i)));
        midNodes = setdiff([1:size(Ai,1)],[[newStartNodes] [newEndNodes]]);
        Ae=Ai(midNodes,midNodes);
        [r c] = find(Ae==1);
        r = r+1; % increase index of every node by one so that node 1 is source node
        c = c+1;
        s = max(Ai(:,[newStartNodes]),[],2);
        t = max(Ai(:,[newEndNodes]),[],2);
        s = s(midNodes,:);
        t = t(midNodes,:);
        [rs cs] = find(s==1);
        [rt ct] = find(t==1);
        r = [r;ones(size(rs,1),1);rs+1;(size(Ae,1)+2)*ones(size(rt,1),1);rt+1];
        c = [c;rs+1;ones(size(rs,1),1);rt+1;(size(Ae,1)+2)*ones(size(rt,1),1)];
        %         r = [r;(size(Ae,1)+1)*ones(size(s,1),1);find(s==1)]
        %         c = [c;find(s==1);(size(Ae,1)+1)*ones(size(s,1),1)]
        %         [flowMatrix mCut] = grMaxFlows([r c ones(size(r,1),1)],[1],[size(Ae,1)+2]);
        Sp = sparse(r,c,1);
        %         S(r,c) = 1;
        F = full(Sp);
        G = [zeros(size(F)),eye(size(F)); F zeros(size(F))];
        [flowval cut R H] = max_flow(sparse(G),1+length(F),length(F));
        if(j==0)
            X = flowval;
        end
        costMat = [costMat; length(find(sCC==i)) length(find(eCC==i)) length(find(S==i)) flowval];
    end
    costMatLog = [costMatLog; costMat j*ones(size(costMat,1),1)];
    if(length(costMat)==0)
        cost = 0;
    else
        cost = sum(costMat(:,4))/(X);
    end
    costLog = [costLog; cost];
    
    
end
% save('syn_v1_rand_40x60','costLog','costMatLog','X','startNodes','endNodes')
figure
plot(costLog);
% [mv, mw] = mincut(A, startNodes);
end

