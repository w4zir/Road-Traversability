close all;

% points = randi(10,2,2);
% 
% d = points(:,1) - points(:,2);
% % d = d/norm(d);
% % n = [d(1),d(2)];
% m = d(2)/d(1)
% n = [1 m]./sqrt(sum(1+m^2));
% 
% p = points(:,1) + 2*transpose(n);
% 
% plot(points(1,:), points(2,:),'b.');
% hold on
% plot(p(1),p(2),'r*')
% 
% axis([-10 20 -10 20])
theta = 80;
configs = [-0.2902    4.1129 theta; 0.6946    3.9392 theta];
% config = [[-3;10] [-2;10]];

% R = [cosd(theta) sind(theta); sind(theta) cosd(theta)];
% configs = [[R*config]' [theta;theta]];
% configs = [0 0 theta; ...
%     -0.25 -0.25 theta; 0 -0.25 theta; 0.25 -0.25 theta; ...
%     -0.25 0 theta; 0.25 0 theta; ...
%     -0.25 0.25 theta; 0 0.25 theta; 0.25 0.25 theta; ...
%     ];

hold on
for i=1:size(configs,1)
    plotVehicle(configs(i,1),configs(i,2),configs(i,3));
end


rectangle('Position',[0,-3,6,6],'Curvature',[1 1])
x =  0.0967742 
y =  0.693069 

plot(x, y ,'r*')

dem_x = 6/120;
dem_y = 20/400;
rectangle('Position',[x-dem_x/2,y-dem_y/2,dem_x,dem_y])