function [ dist ] = distFromLine( p, p1, p2 )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

d1=p2-p1;
d2=p-p1;
% d3=p2-p1;
t = abs(dot(d1,d2))/(norm(d1));
r = sqrt(sum((p-p1).^2));
dist =  sqrt(r^2-t^2);

end

