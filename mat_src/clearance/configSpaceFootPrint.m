function [ output_args ] = configSpaceFootPrint( input_args )
%Plot all valid configurations foot print
%   Detailed explanation goes here

close all; clear; clc;

%% ==================== Part 1: Vehicle info ====================
wheelbase = 1.78;
vehicle_width = 1.58;
track = 1.3;

%% ==================== Part 2: Compute Clearance ====================
load('clearance_roads_quasi.mat');

frame_count = 100;%max(safeMincutCount(:,2));
safetyLog = [];
for frame_id=13:frame_count
    frame_id
    % get frame configs
    idx = find(configsLog(:,7) == frame_id);
    configs = configsLog(idx,1:6);

end

