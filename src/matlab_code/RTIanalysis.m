function [ output_args ] = RTIanalysis( input_args )
%Analyze RTI
%   Detailed explanation goes here

close all;

%% ==================== Part 1: Vehicle info and general parameters ==============
wheelbase = 1.78;
vehicle_width = 1.58;
track = 1.3;

pc_count = 60;
random_configs = 25;
%% ==================== Part 2: Load Data ====================
load('tmp.mat');

%% ==================== Part 3: Analyze randomness effect on nodes count =============
mc_data = reshape(mincutCount(:,1), pc_count, random_configs);
m_mc = mean(mc_data,2);
std_mc = std(mc_data,[],2);
z_mc = (mc_data-repmat(m_mc,1,random_configs))./repmat(std_mc,1,random_configs);
total = pc_count*random_configs - length(z_mc(isnan(z_mc(:))));
first_std = length(find(abs(z_mc(:)) < 1))*100/total
sec_std = length(find(abs(z_mc(:)) < 2))*100/total
third_std = length(find(abs(z_mc(:)) < 3))*100/total

%% ==================== Part 4: Analyze randomness effect on RTI =============
rti = mc_data./repmat(mc_data(1,:),pc_count,1);
m_rti = mean(rti,2);
std_rti = std(rti,[],2);
z_rti = (rti-repmat(m_rti,1,random_configs))./repmat(std_rti,1,random_configs);
total = pc_count*random_configs - length(z_rti(isnan(z_rti(:))));
first_std = length(find(abs(z_rti(:)) < 1))*100/total
sec_std = length(find(abs(z_rti(:)) < 2))*100/total
third_std = length(find(abs(z_rti(:)) < 3))*100/total

end

