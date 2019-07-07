function [ output_args ] = plot_obstacles( obstacles )
%PLOT_OBSTACLES Summary of this function goes here
%   Detailed explanation goes here
hold on

for i=1:size(obstacles,1)
    r = obstacles(i,3);
    d = r*2;
    px = obstacles(i,1)-r;
    py = obstacles(i,2)-r;
    rectangle('Position',[px py d d],'Curvature',[1,1]);
end
end

