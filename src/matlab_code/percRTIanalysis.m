function [ output_args ] = percRTIanalysis( input_args )
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
load('perc_rti_analysis2.mat');

%% ==================== Part 3: Analyze randomness effect on nodes count =============
mean_mc_log = [];
mean_rti_log = [];
std_mc_log = [];
std_rti_log = [];

for perc=0:10:90
    idx = find(mincutCount(:,4) == perc);
    data = mincutCount(idx);
    mc_data = reshape(data(:,1), pc_count, random_configs);
    [z_mc,m_mc,std_mc] = zscore(mc_data,[],2);
    %     m_mc = mean(mc_data,2);
    mean_mc_log = [mean_mc_log m_mc];
    %     std_mc = std(mc_data,[],2);
    %     std_mc_log = [std_mc_log std_mc];
    %     z_mc = (mc_data-repmat(m_mc,1,random_configs))./repmat(std_mc,1,random_configs);
    %     total = pc_count*random_configs - length(z_mc(isnan(z_mc(:))));
    %     first_std = length(find(abs(z_mc(:)) < 1))*100/total
    %     sec_std = length(find(abs(z_mc(:)) < 2))*100/total
    %     third_std = length(find(abs(z_mc(:)) < 3))*100/total
    
    %% ==================== Part 4: Analyze randomness effect on RTI =============
    rti = mc_data./repmat(mc_data(1,:),pc_count,1);
    [z_rti m_rti std_rti] = zscore(rti,[],2);
    %     m_rti = mean(rti,2);
    mean_rti_log = [mean_rti_log m_rti];
    %     std_rti = std(rti,[],2);
    std_rti_log = [std_rti_log std_rti];
    %     z_rti = (rti-repmat(m_rti,1,random_configs))./repmat(std_rti,1,random_configs);
    %     total = pc_count*random_configs - length(z_rti(isnan(z_rti(:))));
    %     first_std = length(find(abs(z_rti(:)) <= 1))*100/total;
    %     sec_std = length(find(abs(z_rti(:)) <= 2))*100/total;
    %     third_std = length(find(abs(z_rti(:)) <= 3))*100/total;
    
    %     mu = 0;
    %     sigma = 1;
    %     pd = makedist('Normal',mu,sigma);
    %     data = abs(z_rti(:));
    %     data2 = sort(data);
    %     data = data2(~isnan(data2));
    %     y = cdf(pd,data);
end
% mean_perc_rti = mean(mean_rti_log,2)
% std_perc_rti = mean(std_rti_log,2)
% % plot box-plot rti of random graphs
% boxplot(mean_rti')
% set(gca,'XLim',[0 60])
% set(gca,'XTick',[0:5:60])
% set(gca,'XTickLabel',[0:5:60])
% xlabel('Pointclouds')
% ylabel('RTI')

% % plot z-score of rti of random graphs
% plot(z_rti,'rs')
% set(gca,'XLim',[0 60])
% set(gca,'XTick',[0:5:60])
% set(gca,'XTickLabel',[0:5:60])
% xlabel('Pointclouds')
% ylabel('RTI')
[z_log m_log std_log] = zscore(mean_rti_log,[],2);
% data = z_log;
data = z_log(2:end,:);
total = numel(data);
first_std = length(find(abs(data(:)) <= 1))/total;
sec_std = length(find(abs(data(:)) <= 2))/total;
third_std = length(find(abs(data(:)) <= 3))/total;
figure
subplot(1,3,1)
histfit(data(:))
title('Normal Distribution')
xlabel('Z-Score')

subplot(1,3,2)
cdfplot(abs(data(:)))
hold on
plot([1,1,0],[0,first_std,first_std],'r--');
plot([2,2,0],[0,sec_std,sec_std],'r--');
plot([3,3,0],[0,third_std,third_std],'r--');
axis([0 3.5 0 1.1])
title('CDF of |Z-Score|')
xlabel('|Z-Score|')
ylabel('CDF')
hold off

subplot(1,3,3)
h = normplot(data(:))
% set(h,'YLim',[0 1])
% set(h,'YTick',[0:0.1:1])
% set(h,'YTickLabel',[0:0.1:1])
xlabel('Z-Score')

% figure

end

