close all
% load('/home/khan/Dropbox/Phd/implementations/traversability/mat_logs/analysis_new.mat');
% 
% mc_tmp = mc_count;
% idx = find(mc_count(:,5) == 100);
% mc_100 = mc_count(idx,:);
% 
% load('/home/khan/Dropbox/Phd/implementations/traversability/mat_logs/analysis_new_node_rem.mat');
% mc_count = [mc_count; mc_100];
% save('/home/khan/Dropbox/Phd/implementations/traversability/mat_logs/analysis_new_node_rem2.mat','mc_count');

nominal_rti = [423 303 130];
nominal_rsi = [409 300 121];
rti_nodes = [200 102 59];
rsi_nodes = [153 99 50];

rti = rti_nodes./nominal_rti

rsi = rsi_nodes./nominal_rsi

figure
bar([rti' rsi'])



% axis off
% Here we preserve the size of the image when we save it.
width = 6;
height = 4;
set(gcf,'InvertHardcopy','on');
set(gcf,'PaperUnits', 'inches');
papersize = get(gcf, 'PaperSize');
left = (papersize(1)- width)/2;
bottom = (papersize(2)- height)/2;
myfiguresize = [left, bottom, width, height];
set(gcf,'PaperPosition', myfiguresize);

% axis([0.5 3.5 0 1])
xticks([1 2 3])
yticks([0 0.25 0.5])
% Save the file as PNG
print('village_pak','-dpng','-r150');