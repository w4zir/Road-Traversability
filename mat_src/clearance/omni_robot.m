% point robot clearance based path computation using roadmap graph
close all
clear all

x_min = -5.5
x_max = 5.5
y_min = -5.5
y_max = 5.5
road_x_min = -5
road_x_max = 5
road_y_min = -5
road_y_max = 5
neighbor_dist = 1
config_count = 4000

% generate obstacles
obs = generate_obstacles(x_min,x_max,y_min,y_max,4);

% Generate roadmap graph
x = (x_max-x_min)*rand(config_count,1) + x_min;
y = (y_max-y_min)*rand(config_count,1) + y_min;
% x(1:2) = 0.4;
% y(1) = 0.4;
% y(2) = -0.4;


[sidx tidx] = meshgrid(1:config_count,1:config_count);
dist = sqrt((x(sidx)-x(tidx)).^2 + (y(sidx)-y(tidx)).^2);
is_neighbor = find(dist <= neighbor_dist & dist > 0);

% adjacency nodes
s_nodes = sidx(is_neighbor);
t_nodes = tidx(is_neighbor);

% configs outside road  boundaries are removed
invalid = find(abs(x)>road_x_max);
invalid_nodes = [invalid];
idx = ismember(s_nodes,invalid);
s_nodes(idx,:) = [];
t_nodes(idx,:) = [];
idx = ismember(t_nodes,invalid);
s_nodes(idx,:) = [];
t_nodes(idx,:) = [];

% configs inside obstacle are removed too
nodes = 1:config_count;
% invalid_nodes = [];
for i=1:size(obs,1)
    dist = sqrt((x(nodes)-obs(i,1)).^2 + (y(nodes)-obs(i,2)).^2);
    invalid = find(dist <= obs(i,3));
    invalid_nodes = [invalid_nodes; invalid];
    idx = ismember(s_nodes,invalid);
    s_nodes(idx,:) = [];
    t_nodes(idx,:) = [];
    idx = ismember(t_nodes,invalid);
    s_nodes(idx,:) = [];
    t_nodes(idx,:) = [];
end


not_neighbor = []
for i=1:size(s_nodes,1)
    for j=1:size(obs,1)
        p1 = [x(s_nodes(i));y(s_nodes(i));0];
        p2 = [x(t_nodes(i));y(t_nodes(i));0];
        p = [obs(j,1);obs(j,2);0];
        is_inside = is_line_in_circle(p,p1,p2,obs(j,3));
        if (is_inside == 1)
            not_neighbor(end+1) = i;
        end
    end
end

s_nodes(not_neighbor) = [];
t_nodes(not_neighbor) = [];

% get valid and invalid nodes
invalid_nodes = unique(invalid_nodes);
valid_nodes = setdiff([1:config_count],invalid_nodes);

% find distance of each valid node to the nearest invalid node
[sidx tidx] = meshgrid(valid_nodes,invalid_nodes);
dist = sqrt((x(sidx)-x(tidx)).^2 + (y(sidx)-y(tidx)).^2);
node_weights = min(dist);


% xnodes = sidx(is_neighbor);
% ynodex = tidx(is_neighbor);
Ds = [[s_nodes, t_nodes, ones(size(s_nodes,1),1)]; [config_count,config_count,0]];
As = spconvert(Ds);
G = graph(As)

% Ds2 = [[s_nodes(not_neighbor), t_nodes(not_neighbor), ones(size(not_neighbor,2),1)]; [config_count,config_count,0]];
% As2 = spconvert(Ds2);
% G2 = graph(As2)


plot(G,'XData',x,'yData',y)
hold on
% plot(G2,'XData',x,'yData',y)
for i=1:size(obs,1)
    r = obs(i,3);
    d = r*2;
    px = obs(i,1)-r;
    py = obs(i,2)-r;
    rectangle('Position',[px py d d],'Curvature',[1,1]);
end
% plot(p1(1),p1(2),'r*')
% plot(p2(1),p2(2),'g*')


% scatter(x,y,'.')
% hold on
% scatter(x(1:2,1),y(1:2,1),'r.')
axis([x_min x_max y_min y_max])

valids = ones(size(x,1),1);
valids(invalid_nodes) = 0;
configs = [x,y,zeros(size(x,1),2),valids];
save('/home/az/git_repos/phd/road-traversability/mat_src/clear4.mat','G','x_min','x_max','y_min','y_max','s_nodes','t_nodes','valid_nodes','invalid_nodes','node_weights','obs','config_count','configs')

function [obstacles] = generate_obstacles (x_min,x_max,y_min,y_max,obs_count)
obstacles = []
obstacles(1,:) = [-3,-2,0.5];
obstacles(2,:) = [-1,-2,0.5];
obstacles(3,:) = [1,0,0.5];
obstacles(4,:) = [3,2,0.5]
% for i=1:obs_count
%     obstacles(end+1,:) = [(x_max-x_min)*rand()+x_min,(y_max-y_min)*rand()+y_min,0.5]
% end
end

function [ is_inside ] = is_line_in_circle( q, p1, p2, rad )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

is_inside = 0;

l1=p2-p1;
l2=q-p1;
l3=q-p2;

d1 = abs(dot(l1,l2))/(norm(l1));
d2 = abs(dot(l1,l3))/(norm(l1));
d3 = sqrt(sum(l1.^2));

r = sqrt(sum((l2).^2));
d =  sqrt(r^2-d1^2);

if (d <= rad && round(d3,2) == round((d1+d2),2))
    is_inside = 1;
end

end



