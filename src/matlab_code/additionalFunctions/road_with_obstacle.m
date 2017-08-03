function [ output_args ] = road_with_obstacle( input_args )
%Generate road with obstacle
%   Detailed explanation goes here

% img = imread('road.jpg');
% 
% figure
% imshow(img)
% 
% axis([0 2000 0 1000])
radius = 0.6;

figure
axis([-11 11 -3 3])
rectangle('Position',[-10 -2 20 4])
hold on
rectangle('Position',[3.04-radius -1.4-radius 2*radius 2*radius],'Curvature',[1 1])
end
