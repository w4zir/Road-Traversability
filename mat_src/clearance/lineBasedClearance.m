function [ safetyLog ] = lineBasedClearance( input_args )
%Compute clearance of road patch using mincut nodes and line equation
%   Detailed explanation goes here

close all; clear;
%% ==================== Part 1: Vehicle info ====================
wheelbase = 1.78;
vehicle_width = 1.58;
track = 1.3;

%% ==================== Part 2: Compute Clearance ====================
load('road_clear_pos_rand_01_10.mat');

frame_count = 50;%max(safeMincutCount(:,2));
safetyLog = [];
for frame_id=43:frame_count
    frame_id
    % get frame configs
    idx = find(configsLog(:,7) == frame_id);
    configs = configsLog(idx,1:6);
    
    % get frame safe mincut nodes
    idx = find(safeMincutLog(:,2) == frame_id);
    mc_idx = safeMincutLog(idx,1);
    mc_configs = configs(mc_idx,:);
    
    % get adjacency data
    idx = find(adjacencyLog(:,4) == frame_id);
    adjacency = adjacencyLog(idx,1:3);
    
    idx = ismember(adjacency(:,1), mc_idx);
    mc_adj = adjacency(idx,1:2);
    idx = ismember(adjacency(:,2), mc_idx);
    mc_adj = [mc_adj; adjacency(idx,1:2)];
    
    % get obstacle info
    idx = find(obstacleLog(:,5) == frame_id);
    obstacles = obstacleLog(idx,1:3);
    
    % compute actual safety value form vehicle width and obstacle info and
    % corresponding line equation
    %     [actual_safety_value, slope intercept] = computeActualClearanceAndLine( obstacles, vehicle_width);
    
    % compute line parameters
    slope = (obstacles(2,2)-obstacles(1,2))/(obstacles(2,1)-obstacles(1,1));
    intercept = obstacles(1,2) - slope*obstacles(1,1);
    a = -slope; b = 1; c = -intercept;
    line = [a;b;c];
    norm_factor = sqrt(a^2+b^2);
    
%     axis([-3 3 -10 10])
%     hold on
% %     plot(obstacles(:,1),obstacles(:,2),'bs')
%     rectangle('Position',[obstacles(1,1)-obstacles(1,3) obstacles(1,2)-obstacles(1,3) 2*obstacles(1,3) 2*obstacles(1,3)],'Curvature',[1 1],'FaceColor','k');
%     rectangle('Position',[obstacles(2,1)-obstacles(2,3) obstacles(2,2)-obstacles(2,3) 2*obstacles(2,3) 2*obstacles(2,3)],'Curvature',[1 1],'FaceColor','k');
%     xt = -4:0.01:4;
%     yt = -a*xt-c;
%     plot(xt,yt)
    % for all mincut node edges, find its intersection with the line
    projectedPoints = [];
    for i=1:size(mc_adj,1)
        edge = [configs(mc_adj(i,1),1:3); configs(mc_adj(i,2),1:3)];
        dist1 = (edge(1,1)*a + edge(1,2)*b + c)/sqrt(a^2+b^2);
        dist2 = (edge(2,1)*a + edge(2,2)*b + c)/sqrt(a^2+b^2);
        %        plot(edge(:,1),edge(:,2),'r.')
        if ((dist1 < 0 & dist2 < 0) || (dist1 > 0 & dist2 > 0))
            continue;
        else
            point = projectEdgeOnLine(line, edge, wheelbase);
            projectedPoints = [projectedPoints; point];
%             plot(edge(:,1),edge(:,2),'b.')
%             plot(point(1),point(2),'g.')
%             dist = (point(1)*a + point(2)*b + c)/norm_factor;
%             if (dist > 0.1)
%                 plot(edge(:,1),edge(:,2),'r.')
%                 plot(point(1),point(2),'gs')
%             end
        end
    end
    
    % compute safety distance by maximizing the distance between pair of
    % projected points on line
    xval = projectedPoints(:,1);
    yval = projectedPoints(:,2);
    [x y] = meshgrid(1:size(projectedPoints,1),1:size(projectedPoints,1));
    dist = sqrt((xval(x)-xval(y)).^2 + (yval(x)-yval(y)).^2);
    [computed_safety_value, maxIndex] = max(dist(:));
    [row, col] = ind2sub(size(dist), maxIndex);
    
    % compute actual safety value form vehicle width and obstacle info
    obstacle_radius = obstacles(1,3) + obstacles(2,3);
    dist_obs = sqrt(sum((obstacles(1,1:2)-obstacles(2,1:2)).^2));
    actual_safety_value = dist_obs - obstacle_radius - vehicle_width;
    
    safetyLog = [safetyLog; [actual_safety_value computed_safety_value]];
    
    G = digraph(spconvert(adjacency));
    xCord = configs(:,1);
    yCord = configs(:,2);
    h = plot(G,'XData',xCord,'yData',yCord,'MarkerSize',3);
    hold on
    axis([-3 3 -10 10])
    plot(mc_configs(:,1),mc_configs(:,2),'r.')
    
    plot(obstacles(:,1),obstacles(:,2),'bs')
    plot(projectedPoints(row,1),projectedPoints(row,2),'gs')
    plot(projectedPoints(col,1),projectedPoints(col,2),'gs')
    rectangle('Position',[obstacles(1,1)-obstacles(1,3) obstacles(1,2)-obstacles(1,3) 2*obstacles(1,3) 2*obstacles(1,3)],'Curvature',[1 1]);
    rectangle('Position',[obstacles(2,1)-obstacles(2,3) obstacles(2,2)-obstacles(2,3) 2*obstacles(2,3) 2*obstacles(2,3)],'Curvature',[1 1]);
    xt = -4:0.1:4;
    yt = -a*xt-c;
    plot(xt,yt)
    plot(projectedPoints(:,1),projectedPoints(:,2),'g.')
    
    plotVehicle(projectedPoints(row,1),projectedPoints(row,2),90+slope*180/pi);
    plotVehicle(projectedPoints(col,1),projectedPoints(col,2),90+slope*180/pi);
    %     for i=1:size(projectedPoints,1)
    %         plotVehicle(projectedPoints(i,1),projectedPoints(i,2),90+slope*180/pi);
    %     end
    hold off
end
% safety_diff = safetyLog(:,1)-safetyLog(:,2);
% error = mean(abs(safety_diff))
% axis([0 1 0 51])
% bar(abs(safety_diff));
% save('/home/wazir/phd_ws/matlab_ws/RTI/logs/clearance_roads_pos_rand_02.mat','safeMincutCount','mincutCount','safeMincutLog','minCutLog','connCompsLog','startNodesLog','goalNodesLog','configsLog','adjacencyLog','obstacleLog','safetyLog')
end

