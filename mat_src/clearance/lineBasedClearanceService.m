function [ safetyLog ] = lineBasedClearanceService( file_name )
%Service that computes clearance of road given data
%   Detailed explanation goes here

%% ==================== Part 1: Vehicle info ====================
wheelbase = 1.78;
vehicle_width = 1.58;
track = 1.3;

%% ==================== Part 2: Compute Clearance ====================
% load('road_clear_pos_rand_01_1.mat');
load(file_name);

frame_count = 50;%max(safeMincutCount(:,2));
safetyLog = [];
for frame_id=1:frame_count
    %     frame_id
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
    
    % compute line parameters
    slope = (obstacles(2,2)-obstacles(1,2))/(obstacles(2,1)-obstacles(1,1));
    intercept = obstacles(1,2) - slope*obstacles(1,1);
    a = -slope; b = 1; c = -intercept;
    line = [a;b;c];
    norm_factor = sqrt(a^2+b^2);
    
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
end
end

