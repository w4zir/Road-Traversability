function [ output_args ] = rsi_safer_path( input_args )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

load('/home/khan/phd_ws/matlab_ws/RTI/logs/rsi_1.mat')
rti1 = mincutCount/max(mincutCount);
plot(rti1);
hold on
rsi1 = safeMincutCount/max(safeMincutCount);
plot(rsi1,'k');

load('/home/khan/phd_ws/matlab_ws/RTI/logs/rsi_2.mat')
rti2 = mincutCount/max(mincutCount);
plot(rti2,'g');
rsi2 = safeMincutCount/max(safeMincutCount);
plot(rsi2,'r');

legend('RTI Route 1','RSI Route 1','RTI Route 2','RSI Route 2')
end

