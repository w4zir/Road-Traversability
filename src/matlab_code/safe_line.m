function [ output_args ] = safe_line( input_args )
%Given a line equation find safety value
%   Detailed explanation goes here

%% Initialization
clear ; close all; clc

%% ==================== Part 1: get required data ====================

load('safe_roads_rand.mat');
frame_id = 8;

wheelbase = 1.78;

% get frame configs
idx = find(configsLog(:,7) == frame_id);
configs = configsLog(idx,1:6);

% get frame safe mincut nodes
idx = find(safeMincutLog(:,5) == frame_id);
mc_configs = safeMincutLog(idx,1:4);

% get adjacency data
idx = find(adjacencyLog(:,4) == frame_id);
adjacency = adjacencyLog(idx,1:3);

idx = ismember(adjacency(:,1), mc_configs(:,1));
mc_adj = adjacency(idx,1:2);
idx = ismember(adjacency(:,2), mc_configs(:,1));
mc_adj = [mc_adj; adjacency(idx,1:2)];

% compute line parameters
p1 = [-3 0];
p2 = [3 0];
safetyLog = [];
for intercept=-1:0.1:1
% slope = (p2(2)-p1(2))/(p2(1)-p1(1))
slope = (mean(mc_configs(:,4))-90)*pi/180;
% intercept = p2(2) - slope*p2(1)
mean(mc_configs(:,4))-90
% compute projection of edge on line
projectedPoints = [];
a = -slope; b = 1; c = -intercept;
line = [a;b;c];
% edge_count = 0;
for i=1:size(mc_adj,1)
    edge = [configs(mc_adj(i,1),1:3); configs(mc_adj(i,2),1:3)];
    dist1 = (edge(1,1)*a + edge(1,2)*b + c)/sqrt(a^2+b^2);
    dist2 = (edge(2,1)*a + edge(2,2)*b + c)/sqrt(a^2+b^2);
    if ((dist1 < 0 & dist2 < 0) || (dist1 > 0 & dist2 > 0))
        continue;
    else
%         edge_count = edge_count+1;
        point = projectEdgeOnLine(line, edge, wheelbase);
        projectedPoints = [projectedPoints; point];
    end
end
% safetyLog = [safetyLog; [intercept edge_count]];
% continue;
% compute safety distance by maximizing the distance between pair of
% projected points on line
xval = projectedPoints(:,1);
yval = projectedPoints(:,2);
[x y] = meshgrid(1:size(projectedPoints,1),1:size(projectedPoints,1));
dist = sqrt((xval(x)-xval(y)).^2 + (yval(x)-yval(y)).^2);
[safety_value, maxIndex] = max(dist(:));
[row, col] = ind2sub(size(dist), maxIndex);
% plot(points(row,1),points(row,2),'r*');
% plot(points(col,1),points(col,2),'r*');
safety_value
safetyLog = [safetyLog; [intercept safety_value]];
end
plot(safetyLog(:,1),safetyLog(:,2),'bs');
% % get valid and invalid nodes
% valid_idx = find(configs(:,5)==1);
% valid = configs(valid_idx,:);
% invalid_idx = find(configs(:,5)==0);
% invalid = configs(invalid_idx,:);
% 
% %% Plot data
% % plot all configs
% plot(configs(:,1),configs(:,2),'b.');
% hold on
% % plot valid and invalid configs
% plot(valid(:,1),valid(:,2),'g.');
% plot(invalid(:,1),invalid(:,2),'r.');
% % plot safe mincut
% % plot(mc_configs(:,2),mc_configs(:,3),'ko');
% plot(mc_configs(:,2),mc_configs(:,3),'ko','MarkerSize',7,'MarkerFaceColor','g');
% % plot line
% plot([p1(1);p2(1)],[p1(2);p2(2)],'LineWidth',1,'Color','K');
% % plot projected points on line
% plot(projectedPoints(:,1), projectedPoints(:,2),'rs');


end

