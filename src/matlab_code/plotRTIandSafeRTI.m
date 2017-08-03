function [ output_args ] = plotRTIandSafeRTI( input_args )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

load('logs/long_road_quasi2.mat');

RTI = costLog/maxVal;
safeRTI = safeCostLog/maxSafeValue;

rti_plot = [RTI safeRTI];

bar(rti_plot);
legend ('RTI', 'RSI')

end

