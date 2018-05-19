function [ output_args ] = plotDEM( input_args )
%Plot DEM cells and vehicle to check collision
%   Detailed explanation goes here

close all;

file = 'dem_info.txt';
dems = importdata(file,' ');

theta = 70;
configs = [-1.0 0 theta];

hold on
for i=1:size(configs,1)
    plotVehicle(configs(i,1),configs(i,2),configs(i,3));
end
rectangle('Position',[0,-3,6,6],'Curvature',[1 1])

pos_dem = find(dems(:,4) == 1);
neg_dem = find(dems(:,4) == -1);

plot(dems(:,1),dems(:,2),'g.');
plot(dems(neg_dem,1),dems(neg_dem,2),'b.');
plot(dems(pos_dem,1),dems(pos_dem,2),'r.');

x =  0.387097 
y =  1.28713

plot(x, y ,'k*')


end

