function [ dist ] = configDistance( x1,y1,t1, x2,y2,t2, L)
%Distance find distance between two configurations
%   Detailed explanation goes here

t1 = t1*pi/180;
t2 = t2*pi/180;
A = L*L;
dcita = t1 - t2;
if (dcita>pi)
    dcita = dcita + 2*pi;
elseif (dcita<-pi)
    dcita = dcita + 2*pi;
end
d = (x1-x2)*(x1-x2)+ (y1-y2)*(y1-y2)+ A*dcita*dcita;
dist = d^0.5;

end

