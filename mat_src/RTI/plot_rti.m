
close all
clear all

load('/home/khan/Dropbox/Phd/implementations/traversability/mat_logs/kitti_86_2.mat')

% % select top five
% [~, idx]= sort(avg_rti,'descend');
% top_idx = idx(1:5);
%
% % select bottom five
% [~, idx]= sort(avg_rti,'ascend');
% bot_idx = idx(1:5);

% % plot
% bar([avg_rti(top_idx) avg_rti(bot_idx)])
% figure
% subplot(2,1,1)
% plot selected rti
% bar(rti_log')
% idc = [13,26,36];  % count
% idc = [19,11,3]; % position
% idc = [9,3,10]; % size
% idc = [2,3,5]
% bar(rti_log(2:4,idc)')
% xticks({})
% yticks([0 0.15 0.3])
% yticklabels({});
% 
% subplot(2,1,2)
% bar(rsi_log(2:4,idc)')
% xticks({})
% yticks([0 0.15 0.3])
% yticklabels({});

% axis([0.6 3.3 0 0.3])
% rti_log = rti_log(:,idc);

%==========================================================================%
% %plot line markers
% figure
% 
% plot(rti_log(1,:),'ro','MarkerFaceColor','r','MarkerSize',7)
% hold on
% plot(rti_log(2,:),'g^','MarkerFaceColor','g','MarkerSize',7)
% plot(rti_log(3,:),'b*','MarkerFaceColor','b','MarkerSize',7)
% plot(rti_log(4,:),'ms','MarkerFaceColor','m','MarkerSize',7)

% mins = min(rti_log,[],1);
% maxs = max(rti_log,[],1);
% idx = [1:size(rti_log,2)];
% 
% plt_array = zeros(size(rti_log,2)*2 + size(rti_log,2) - 1,2);
% plt_array(3*(idx-1)+1,:) = [idx; mins]';
% plt_array(3*(idx-1)+2,:) = [idx; maxs]';
% plt_array(3*(idx(1:size(rti_log,2)-1)-1)+3,:) = repmat([NaN NaN],size(rti_log,2) - 1,1);
% 
% plot(plt_array(:,1),plt_array(:,2),'k')
%==========================================================================%
frames = [1:size(avg_rti,2)]
plot(rti_log(1,frames),'r-','LineWidth',1)
hold on
plot(rti_log(2,frames),'g-','LineWidth',1)
plot(rti_log(3,frames),'b-','LineWidth',1)
plot(rti_log(4,frames),'m-','LineWidth',1)


axis([0 size(rti_log,2)+1 0 1.04])
% label and legend
% xlabel('Road Patch')
% ylabel('RTI')
% legend('Vehicle1', 'Vehicle2', 'Vehicle3','Vehicle4')

% axis([0 26 0 1.1])
% % yticks([0 0.1 0.3 0.5 0.7]) % obs_pos
% % yticks([0 0.2 0.4 0.6]) % obs_size
% yticks([0 0.2 0.4]) % obs_count
% yticks([0 0.5 1])
% xticks([50 100 150 200])
% yticklabels({});
% xticklabels({});
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

%Save the file as PNG
% print('/home/khan/Pictures/matlab_imgs/village_2.png','-dpng','-r150');
