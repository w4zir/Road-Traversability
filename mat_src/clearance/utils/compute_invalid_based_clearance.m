function [ d_min ] = compute_( configs, valid_nodes, invalid_nodes )
% Valid configs clearance using invalid configs
%   Detailed explanation goes here

% angle difference weightage
k1 = 10;
k2 = 100
fprintf(strcat('Angle Diff Weight: ',int2str(k1),'|',int2str(k2),'\n'))

% total configs
total_configs = size(configs,1);

valid_x = configs(valid_nodes,1);
valid_y = configs(valid_nodes,2);
valid_t = configs(valid_nodes,3)*pi/180.0;

invalid_x = configs(invalid_nodes,1);
invalid_y = configs(invalid_nodes,2);
invalid_t = configs(invalid_nodes,3)*pi/180.0;

% x difference
[v, iv] = meshgrid(valid_x, invalid_x);
dx = v - iv;

% y difference 
[v, iv] = meshgrid(valid_y, invalid_y);
dy = v - iv;

% theta difference
[v, iv] = meshgrid(valid_t, invalid_t);
dt = v - iv;

% compite valid nodes to all invalid nodes distances
d_v2iv = sqrt(k1*dx.^2 + dy.^2 + k2*dt.^2);

% compute minimum distance of all invalid nodes
d_min = min(d_v2iv,[],1);

end

