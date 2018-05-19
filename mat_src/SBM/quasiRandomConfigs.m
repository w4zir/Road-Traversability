function [ output_args ] = generateQuasiRandomConfigs( input_args )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
close all

% x_coord = [-3:0.25:3];
% y_coord = [-10:0.25:10];
xmin = -3;
xmax = 3;
ymin = -10;
ymax = 10;
d = 0.25;
x_coord = [-9:d:9];
y_coord = [-20:d:20];

[xc yc] = meshgrid(x_coord,y_coord);
xc = reshape(xc,1,numel(xc));
yc = reshape(yc,1,numel(yc));

tconfigs = [xc;yc];

configs = [tconfigs' 90*ones(length(tconfigs),1)];
configs = [];
nconfigs = [];


% for theta = 90:10:90
% yd = d/cosd(theta);
% 
% for y=ymin:yd:ymax
%         c = y + tand(theta)*(-2);
%     xp = xmin;
%     yp = y;
%     idx = 1;
%     while(xp >= xmin & xp <= xmax & yp >= ymin & yp <=ymax)
%         nconfigs = [nconfigs; [xp yp]];
%         xp = xmin + (idx*d)*cosd(90-theta);
%         yp = y - (idx*d)*sind(90-theta);
%         idx = idx + 1;
%     end
% end
% 
% xd = d/cosd(theta);
% for x=xmin:xd:xmax
%         c = y + tand(theta)*(-2);
%     xp = x;
%     yp = ymax;
%     idx = 1;
%     while(xp >= xmin & xp <= xmax & yp >= ymin & yp <=ymax)
%         nconfigs = [nconfigs; [xp yp]];
%         xp = x + (idx*d)*cosd(90-theta);
%         yp = ymax - (idx*d)*sind(90-theta);
%         idx = idx + 1;
%     end
% end
% configs =[configs; [nconfigs (theta)*ones(length(nconfigs),1)]];

for theta=45:10:45

    R = [cosd(theta-90) -sind(theta-90); sind(theta-90) cosd(theta-90)];
    nconfigs = R*tconfigs;
    configs =[configs; [nconfigs' (theta)*ones(length(nconfigs),1)]];
% end

hold on 

c = ymax/2 + tand(90-theta)*(xmin);
x = -5:0.01:5;
y = -tand(90-theta)*x + c;
plot(x,y)

xp = xmin + d*cosd(90-theta);
yp = ymax + d*sind(90-theta);
c = yp + tand(90-theta)*(xp);
y = -tand(90-theta)*x + c;
plot(x,y)
end
% 
% idx = find(configs(:,1)<xmin | configs(:,1)>xmax);
% configs(idx,:) = [];
% idx = find(configs(:,2)<ymin | configs(:,2)>ymax);
% configs(idx,:) = [];

% figure
configs90 = find(configs(:,3)==90);
configs100 = find(configs(:,3)~=(90));
x_prm = configs(:,1);
y_prm = configs(:,2);
t_prm = configs(:,3);
u_prm = cosd(t_prm);
v_prm = sind(t_prm);
quiver(x_prm(configs90),y_prm(configs90),u_prm(configs90),v_prm(configs90),0.5,'g')
hold on
quiver(x_prm(configs100),y_prm(configs100),u_prm(configs100),v_prm(configs100),0.5,'r')

plot(configs(:,1),configs(:,2),'b*')

rectangle('Position',[-3 -10 6 20])

axis([-5 5 -12 12])

figure
histogram(configs(:,3))
end

