%% Estimating Road Clearance Using Gradient Descent


%% Initialization
clear ; close all; clc

wheelbase = 1.78;
vehicle_width = 1.58;
track = 1.3;

%% ==================== Part 1: Load Data ====================

load('road_clear_pos_rand_01_10.mat');
frame_id = 7 %randi(100,1,1)

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

% get start nodes
idx = find(startNodesLog(:,2) == frame_id);
startNodes = startNodesLog(idx,1);

% get goal nodes
idx = find(goalNodesLog(:,2) == frame_id);
goalNodes = goalNodesLog(idx,1);

% get obstacle info
idx = find(obstacleLog(:,5) == frame_id);
obstacles = obstacleLog(idx,1:3);

% Publish road with obstacle and narrow passage
figure
axis([-4 4 -11 11])
hold on
plot(obstacles(:,1),obstacles(:,2),'bs')
rectangle('Position',[obstacles(1,1)-obstacles(1,3) obstacles(1,2)-obstacles(1,3) 2*obstacles(1,3) 2*obstacles(1,3)],'Curvature',[1 1]);
rectangle('Position',[obstacles(2,1)-obstacles(2,3) obstacles(2,2)-obstacles(2,3) 2*obstacles(2,3) 2*obstacles(2,3)],'Curvature',[1 1]);
plot(obstacles(:,1),obstacles(:,2));

% Publish Mincut
figure
G = digraph(spconvert(adjacency));
xCord = configs(:,1);
yCord = configs(:,2);
h = plot(G,'XData',xCord,'yData',yCord,'MarkerSize',3);
hold on
axis ([-3.5 3.5 -11 11])
valid_idx = find(configs(:,5)==1);
valid = configs(valid_idx,:);
invalid_idx = find(configs(:,5)==0);
invalid = configs(invalid_idx,:);

plot(valid(:,1),valid(:,2),'g.');
plot(invalid(:,1),invalid(:,2),'r.');

plot(mc_configs(:,1),mc_configs(:,2),'s','MarkerSize',7,'MarkerFaceColor','g');

plot(configs(startNodes,1),configs(startNodes,2),'bs');
plot(configs(goalNodes,1),configs(goalNodes,2),'ks');

legend('Configs Connection', 'Valid Configs', 'Invalid Configs', 'Mincut Configs', 'Start Configs', 'Goal Configs')
xlabel('Width')
ylabel('Length')

%% =================== Part 2: Safety Assuming Known Line Parameters ==================

% compute line parameters
slope = (obstacles(2,2)-obstacles(1,2))/(obstacles(2,1)-obstacles(1,1));
intercept = obstacles(1,2) - slope*obstacles(1,1);
a = -slope; b = 1; c = -intercept;
line = [a;b;c];

% get mincut adjacency data
idx = ismember(adjacency(:,1), mc_idx);
mc_adj = adjacency(idx,1:3);
idx = ismember(adjacency(:,2), mc_idx);
mc_adj = [mc_adj; adjacency(idx,1:3)];

% Publish Mincut and line over it
% G = digraph(spconvert(adjacency));
% xCord = configs(:,1);
% yCord = configs(:,2);
% h = plot(G,'XData',xCord,'yData',yCord,'MarkerSize',3);
% hold on
% axis ([-3.5 3.5 -11 11])
% valid_idx = find(configs(:,5)==1);
% valid = configs(valid_idx,:);
% invalid_idx = find(configs(:,5)==0);
% invalid = configs(invalid_idx,:);
%
% plot(valid(:,1),valid(:,2),'g.');
% plot(invalid(:,1),invalid(:,2),'r.');
%
% plot(mc_configs(:,1),mc_configs(:,2),'s','MarkerSize',7,'MarkerFaceColor','g');
%
% plot(configs(startNodes,1),configs(startNodes,2),'bs');
% plot(configs(goalNodes,1),configs(goalNodes,2),'ks');
%
% xt = -4:0.1:4;
% yt = slope*xt + intercept;
% plot(xt,yt,'LineWidth',3,'Color','k')
% legend('Configs Connection', 'Valid Configs', 'Invalid Configs', 'Mincut Configs', 'Start Configs', 'Goal Configs')
% xlabel('Width')
% ylabel('Length')
% plot mincut adjacency subgraph
% figure
% G = digraph(spconvert(mc_adj));
% xCord = mc_configs(:,1);
% yCord = mc_configs(:,2);
% h = plot(G,'XData',xCord,'yData',yCord,'MarkerSize',3);
% hold on
% axis ([-3.5 3.5 -11 11])
% valid_idx = find(configs(:,5)==1);
% valid = configs(valid_idx,:);
% invalid_idx = find(configs(:,5)==0);
% invalid = configs(invalid_idx,:);
%
% plot(valid(:,1),valid(:,2),'g.');
% plot(invalid(:,1),invalid(:,2),'r.');
%
% plot(mc_configs(:,1),mc_configs(:,2),'s','MarkerSize',7,'MarkerFaceColor','g');
%
% plot(configs(startNodes,1),configs(startNodes,2),'bs');
% plot(configs(goalNodes,1),configs(goalNodes,2),'ks');
%
% legend('Configs Connection', 'Valid Configs', 'Invalid Configs', 'Mincut Configs', 'Start Configs', 'Goal Configs')
% xlabel('Width')
% ylabel('Length')

% for all mincut node edges, find its intersection with the line
projectedPoints = [];
for i=1:size(mc_adj,1)
    if (i == 4350)
        asda = 1;
    end
    edge = [configs(mc_adj(i,1),1:3); configs(mc_adj(i,2),1:3)];
    dist1 = (edge(1,1)*a + edge(1,2)*b + c)/sqrt(a^2+b^2);
    dist2 = (edge(2,1)*a + edge(2,2)*b + c)/sqrt(a^2+b^2);
    %         plot(edge(1,:),edge(2,:),'r.')
    if ((dist1 < 0 & dist2 < 0) || (dist1 > 0 & dist2 > 0))
        continue;
    else
        point = projectEdgeOnLine(line, edge, wheelbase);
        projectedPoints = [projectedPoints; [point i]];
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

safety_diff_with_known_line = computed_safety_value - actual_safety_value
% plot data
figure
axis([-4 4 -11 11])
hold on
plot(obstacles(:,1),obstacles(:,2),'bs')
plot(projectedPoints(row,1),projectedPoints(row,2),'gs')
plot(projectedPoints(col,1),projectedPoints(col,2),'gs')
rectangle('Position',[obstacles(1,1)-obstacles(1,3) obstacles(1,2)-obstacles(1,3) 2*obstacles(1,3) 2*obstacles(1,3)],'Curvature',[1 1]);
rectangle('Position',[obstacles(2,1)-obstacles(2,3) obstacles(2,2)-obstacles(2,3) 2*obstacles(2,3) 2*obstacles(2,3)],'Curvature',[1 1]);
xt = -4:0.1:4;
yt = -a*xt-c;
plot(xt,yt)
plot(projectedPoints(:,1),projectedPoints(:,2),'g.')

h1 = openfig('data/road_clearance_pos_rand_01-2.fig','reuse'); % open figure
ax1 = gca; % get handle to axes of figure
% return
%% =================== Part 3: Gradient descent ===================
fprintf('Running Gradient Descent ...\n')
% set data for gradient descent
data = mc_configs;
X = data(:, 1); y = data(:, 2); slope = mean(data(:,3))-90
m = length(y); % number of training examples

X = [ones(m, 1), data(:,2)]; % Add a column of ones to x
% theta = [0; 0]; % initialize fitting parameters
theta = [0; tand(slope)]; % initialize fitting parameters

% Some gradient descent settings
iterations = 1000;
alpha = 0.01;

% compute and display initial cost
computeCost(X, y, theta)

% run gradient descent
theta = gradientDescent(X, y, theta, slope, alpha, iterations)

% print theta to screen
fprintf('Theta found by gradient descent: ');
fprintf('%f %f \n', theta(1), theta(2));

% Plot the linear fit
hold on; % keep previous plot visible
plot(X(:,2), X*theta, '-')
legend('Training data', 'Linear regression')
hold off % don't overlay any more plots on this figure


%% ============= Part 4: Compute safety distance using projection =============
safe_mc = data(:,2:4);
norm_factor = sqrt(theta(2)^2 + 1);
a = -theta(2); b=1; c=-theta(1);
% a = 0; b=1; c=-0.5;
point_dist_to_line = (safe_mc(:,1:2)*[a;b] + c)/norm_factor;
idx = find(abs(point_dist_to_line) < 0.5);

% point_dist_to_line = point_dist_to_line(idx);
% safe_mc = safe_mc(idx,:);
idx1 = find(point_dist_to_line < 0);
idx2 = find(point_dist_to_line >= 0);
mc1 = safe_mc(idx1,:);
mc2 = safe_mc(idx2,:);
mean(mc1(:,3))
mean(mc2(:,3))

% points = safe_mc - dist*[cosd(90+slope) sind(90+slope)];
points = safe_mc(:,1:2) + [point_dist_to_line.*[cosd(safe_mc(:,3))./(sind(slope-safe_mc(:,3)))]  point_dist_to_line.*[sind(safe_mc(:,3))./(sind(slope-safe_mc(:,3)))]];
hold on
plot(points(:,1),points(:,2),'bo');
xval = points(:,1);
yval = points(:,2);
[x y] = meshgrid(1:numel(point_dist_to_line),1:numel(point_dist_to_line));
dist = sqrt((xval(x)-xval(y)).^2 + (yval(x)-yval(y)).^2);
% [max_val] = max(max(dist))
[safety_value, maxIndex] = max(dist(:));
[row, col] = ind2sub(size(dist), maxIndex);
% plot(points(row,1),points(row,2),'r*');
% plot(points(col,1),points(col,2),'r*');
safety_value

% %% ============= Part 4: Compute alternative safety distance using projection =============
% safe_mc = data(:,2:3);
% norm_factor = sqrt(theta(2)^2 + 1);
% a = -theta(2); b=1; c=-theta(1);
% dist = (safe_mc*[a;b] + c)/norm_factor;
%
% points = safe_mc - dist*[cosd(90+slope) sind(90+slope)];
% hold on
% plot(points(:,1),points(:,2),'bo');
% xval = points(:,1);
% yval = points(:,2);
% [x y] = meshgrid(1:m,1:m);
% dist = sqrt((xval(x)-xval(y)).^2 + (yval(x)-yval(y)).^2);
% % [max_val] = max(max(dist))
% [maxNum, maxIndex] = max(dist(:));
% [row, col] = ind2sub(size(dist), maxIndex);
% % plot(points(row,1),points(row,2),'r*');
% % plot(points(col,1),points(col,2),'r*');
% maxNum

%% ============= plot =============
configs = safe_mc;
x_prm = configs(:,1);
y_prm = configs(:,2);
t_prm = configs(:,3);
u_prm = cosd(t_prm);
v_prm = sind(t_prm);
quiver(x_prm,y_prm,u_prm,v_prm,0.5,'g')

