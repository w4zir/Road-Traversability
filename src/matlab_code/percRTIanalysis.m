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
load('/home/khan/Dropbox/Phd/implementations/matlab_ws/RTI/logs/perc_rti_analysis2.mat');

%% ==================== Part 3: Analyze randomness effect on nodes count =============
mean_mc_log = [];
mean_rti_log = [];
std_mc_log = [];
std_rti_log = [];
rti_log = [];

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
    rti_log = [rti_log rti];
    [z_rti m_rti std_rti] = zscore(rti,[],2);
    plot(std_rti)
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
[z_log m_log std_log] = zscore(rti_log,[],2);
% data = z_log;

%==================== plot z-score =========================
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

%==================== plot std histogram =========================
close all

figure
% bar(mean(std_rti_log(2:end,:),1))
% axis([0 11 0 0.032])
% yticks([0 0.01 0.02 0.03])
% xticks([1 5 10])
% yticklabels({});
% xticklabels({});
% axis([0 1 0 11])
% yticks([0 5 10])
% % xticks([0.05 0.45 0.95])
histogram(std_log(2:end))
% axis([0 1 0 11])
% yticks([0 5 10])
% xticks([0.05 0.45 0.95])
xticklabels({});
yticklabels({});

%==================== plot std heatmap =========================
close all
figure
[val sort_idx] = sort(m_log)
std_perc_sorted = std_rti_log(sort_idx,:);
h = heatmap(std_perc_sorted(1:59,:));
% h.yLabels = {}
% xticks([1 5 10])
% yticklabels({});
% xticklabels({});

%==================== plot z-score =========================
% close all
% figure
% histogram(m_log,10)
% axis([0 1 0 11])
% yticks([0 5 10])
% % xticks([0.05 0.45 0.95])
% yticklabels({});
% xticklabels({});

% Here we preserve the size of the image when we save it.
width = 4;
height = 3;
set(gcf,'InvertHardcopy','on');
set(gcf,'PaperUnits', 'inches');
papersize = get(gcf, 'PaperSize');
left = (papersize(1)- width)/2;
bottom = (papersize(2)- height)/2;
myfiguresize = [left, bottom, width, height];
set(gcf,'PaperPosition', myfiguresize);

%Save the file as PNG
% print('/home/khan/Pictures/matlab_imgs/perc_rti_std_heatmap.png','-dpng','-r150');

end

