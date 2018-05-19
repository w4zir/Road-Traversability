function [ output_args ] = dubinsCurve( input_args )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

close all

xmin = -2;
xmax = 2;
ymin = -3;
ymax = 3;

x = 0;
y = 0;
r = 1;%2.62;
xrc = x+r;
xlc = x-r;
yrc = 0;
ylc = 0;

phi = 0;

xd = 0.25;
yd = 0.25;

xp = [ xmin:xd:0 0:xd:xmax];
yp = [ ymin:yd:0 0:yd:ymax];
xpp = repmat(xp',size(yp));
ypp = repmat(yp',size(xp));
% xp = repmat([x-xd,x, x+xd],1,3);
% yp = [ones(1,3)*(y-yd),ones(1,3)*y, ones(1,3)*(y+yd)];

t = 0:1:360;
xrc = r*cosd(phi);
yrc = r*sind(phi);
xlc = r*cosd(180+phi);
ylc = r*sind(180+phi);

% axis([-5 5 -5 -5]);

xr = xrc*ones(size(t)) + r*cosd(t);
yr = yrc*ones(size(t)) + r*sind(t);

xl = xlc*ones(size(t)) + r*cosd(t);
yl = ylc*ones(size(t)) + r*sind(t);

hold on
plot(xr,yr)
plot(xl,yl)
plot(xpp,ypp','r*')
plot(0,0','go')

end

