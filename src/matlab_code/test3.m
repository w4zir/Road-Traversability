load('/home/khan/Dropbox/Phd/implementations/traversability/mat_logs/analysis_new.mat');

mc_tmp = mc_count;
idx = find(mc_count(:,5) == 100);
mc_100 = mc_count(idx,:);

load('/home/khan/Dropbox/Phd/implementations/traversability/mat_logs/analysis_new_node_rem.mat');
mc_count = [mc_count; mc_100];
save('/home/khan/Dropbox/Phd/implementations/traversability/mat_logs/analysis_new_node_rem2.mat','mc_count');