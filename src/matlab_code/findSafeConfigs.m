function [ output_args ] = findSafeConfigs( input_args )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
close all
load('minccut_configs');
% [theta,rho] = ransac(configs(1:2,:),10,0.01,0);
clusters = clusterdata(mc_configs(:,1:2),'criterion','distance','cutoff',0.7);
[c_freq,c_id]=hist(clusters,unique(clusters))
plot(mc_configs(:,1),mc_configs(:,2),'b*');
hold on
[~,max_c_idx] = max(c_freq);
max_c_id = c_id(max_c_idx);
idx = find(clusters==max_c_id);
plot(mc_configs(idx,1),mc_configs(idx,2),'r*');

c_configs = mc_configs(idx,:);

validIdx = find(configs(:,5)==1);
valid_configs = configs(validIdx,:);

safest_distance = 1000;
safest_points = [];


[min_vals] = min(mc_configs(idx,1:2),[],1);
[max_vals] = max(mc_configs(idx,1:2),[],1);

safest_distance = max(max_vals - min_vals)

plot(configs(:,1),configs(:,2),'r.','MarkerSize',4);
hold on
plot(valid_configs(:,1),valid_configs(:,2),'b.','MarkerSize',4);
plot(mc_configs(:,1),mc_configs(:,2),'g*');
%     plot(c_configs(i,1),c_configs(i,2),'r*');

%%
% For each config in mc_configs find the line it is placed on, and then find
% the distance of other configs in mc_configs from it. take the line with
% that minimizes the distance and find its safety
% slope = 0;
% intercept = 0;
% max_total_dist = 10000;
% for i=1:size(c_configs,1)
%     hold off
%     plot(configs(:,1),configs(:,2),'r.','MarkerSize',4);
%     hold on
%     plot(valid_configs(:,1),valid_configs(:,2),'b.','MarkerSize',4);
%     plot(mc_configs(:,1),mc_configs(:,2),'g*');
%     plot(c_configs(i,1),c_configs(i,2),'r*');
%
%     t_slope = tand(c_configs(i,3)-90);
%     t_intercept = c_configs(i,2) - t_slope*c_configs(i,1);
%
%     n = [-t_slope 1]./sqrt(sum(1+t_slope^2));
%     dist = n*transpose(repmat(c_configs(i,1:2),size(c_configs,1),1)-c_configs(:,1:2));
%     total_dist = sum(abs(dist));
%     if (total_dist < max_total_dist)
%         max_total_dist = total_dist;
%         slope = t_slope;
%         intercept = t_intercept;
%     end
%
%     %     dist = c_configs(:,2) - slope * c_configs(:,1) - intercept;
%     %     idx = find(abs(dist) < 0.0001);
%     %     pssible_safe_configs = c_configs(idx,1:3);
% end
% x = -3:0.01:3;
% y = slope*x + intercept;
% plot(x,y)

%%
% find minimum ditance
% for i=1:size(c_configs,1)
%     slope = tand(c_configs(i,3)-90);
%     intercept = c_configs(i,2) - slope*c_configs(i,1);
%
%     x = -3:0.01:3;
%     y = slope*x + intercept;
%
%     %      n = [-slope 1]./sqrt(sum(1+slope^2));
%     %      dist = n*transpose(repmat(c_configs(i,1:2),size(valid_configs,1),1)-valid_configs(:,1:2));
%     dist = valid_configs(:,2) - slope * valid_configs(:,1) - intercept;
%     idx = find(abs(dist) < 0.0001);
%     pssible_safe_configs = valid_configs(idx,1:3);
%
%     safe_clusters = clusterdata(pssible_safe_configs,'criterion','distance','cutoff',0.5);
%     p_idx = find(pssible_safe_configs(:,1)==c_configs(i,1) & pssible_safe_configs(:,2)==c_configs(i,2));
%     p_cluster = safe_clusters(p_idx);
%     p_c_idx = find(safe_clusters==p_cluster);
%     safe_configs = pssible_safe_configs(p_c_idx,:);
%
%
%     %     [min_x minIdx] = min(safe_configs(:,1));
%     %     [max_x maxIdx] = max(safe_configs(:,1));
%     %     min_point = safe_configs(minIdx,1:2);
%     %     max_point = safe_configs(maxIdx,1:2);
%     %
%     %     safe_distance = sqrt(sum((max_point-min_point).^2));
%     %     if(safe_distance < safest_distance)
%     %         safest_distance = safe_distance;
%     %         safest_points = [min_point;max_point];
%     %     end
%     hold off
%     plot(configs(:,1),configs(:,2),'r.','MarkerSize',3);
%     hold on
%     plot(valid_configs(:,1),valid_configs(:,2),'b.','MarkerSize',3);
%     plot(mc_configs(:,1),mc_configs(:,2),'g*');
%     plot(c_configs(i,1),c_configs(i,2),'r*');
%     %     plot(x,y);
%     plot(valid_configs(idx,1),valid_configs(idx,2),'k*');
%
%     %     plot(safest_points(:,1),safest_points(:,2));
%     %     point = [c_configs(i,1:2) + 1*n];
%     %     plot(point(1),point(2),'b*');
% end
% hold on
% plot(safest_points(:,1),safest_points(:,2));
% safest_distance
% angles = unique(mc_configs(:,3));
% angle_freq = [];

%%
% for angles with maximum configurations in mincut find the line passing
% through same angle configurations that has the least safety

angles = unique(c_configs(:,3));
[c_freq,c_id]=hist(c_configs(:,3),angles);
[max_val,max_c_idx] = max(c_freq);
max_mc_theta_idx = find(c_freq == max_val);


safest_distance = 1000;
safest_points = [];
for i=1:numel(max_mc_theta_idx)
    % find a line using ransace that fits maximum number of configurations
    % in mincut nodes for theta having maximum number of configs in mincut
    mc_theta = c_id(max_mc_theta_idx(i));
    mc_theta_idx = find(c_configs(:,3) == mc_theta);
    theta_configs = c_configs(mc_theta_idx,:);
    
    for j=1:size(theta_configs,1);
        t_slope = tand(theta_configs(j,3)-90);
        t_intercept = theta_configs(j,2) - t_slope*theta_configs(j,1);
        
        % distance of all valid configurations from the fitted line. Then find
        % all valid configurations that have same theta and fall on the line
        n = [-t_slope 1]./sqrt(sum(1+t_slope^2));
        dist = n*transpose(repmat(theta_configs(j,1:2),size(valid_configs,1),1)-valid_configs(:,1:2));
        idx = find(abs(dist) < 0.0001);
        t_configs =  valid_configs(idx,:);
        idx = find(t_configs(:,3) == mc_theta);
        pssible_safe_configs = t_configs(idx,1:3);
        safe_configs_clusters = clusterdata(pssible_safe_configs(:,1:2),'criterion','distance','cutoff',0.5);
        
        [ct_freq,ct_id]=hist(safe_configs_clusters,unique(safe_configs_clusters));
        [~,max_ct_idx] = max(ct_freq);
        max_ct_id = ct_id(max_ct_idx);
        idx = find(safe_configs_clusters==max_ct_id);
        
        possible_config_cluster = pssible_safe_configs(idx,:);
        [min_x minIdx] = min(possible_config_cluster(:,1));
        [max_x maxIdx] = max(possible_config_cluster(:,1));
        min_point = possible_config_cluster(minIdx,1:2);
        max_point = possible_config_cluster(maxIdx,1:2);
        
        safe_distance = sqrt(sum((max_point-min_point).^2));
        if(safe_distance < safest_distance)
            safest_distance = safe_distance;
            safest_points = [min_point;max_point];
        end
        hold off
        plot(configs(:,1),configs(:,2),'r.','MarkerSize',5);
        hold on
        plot(valid_configs(:,1),valid_configs(:,2),'b.','MarkerSize',5);
        plot(mc_configs(:,1),mc_configs(:,2),'g*');
        plot(safest_points(:,1),safest_points(:,2),'rs');
        plot(safest_points(:,1),safest_points(:,2),'LineWidth',5);
        x = -3:0.01:3;
        y = t_slope*x + t_intercept;
        plot(x,y)
    end
end
safest_distance

%%
% for angles with maximum configurations in mincut find the line passing
% through same angle configurations that has the least safety

% angles = unique(c_configs(:,3));
% [c_freq,c_id]=hist(c_configs(:,3),angles);
% [max_val,max_c_idx] = max(c_freq);
% max_mc_theta_idx = find(c_freq == max_val);
% 
% 
% safest_distance = 1000;
% safest_points = [];
% for i=1:numel(max_mc_theta_idx)
%     % find a line using ransace that fits maximum number of configurations
%     % in mincut nodes for theta having maximum number of configs in mincut
%     mc_theta = c_id(max_mc_theta_idx(i));
%     mc_theta_idx = find(c_configs(:,3) == mc_theta);
%     
% %     theta_configs = transpose(c_configs(mc_theta_idx,1:2));
% %     [theta,rho] = ransacForSameTheta(theta_configs,20,0.01,0,mc_theta);
% % 
% %     % distance of all valid configurations from the fitted line. Then find
% %     % all valid configurations that have same theta and fall on the line
% %     dist = [sin(theta) cos(theta)]*transpose(valid_configs(:,1:2)) - rho;
% %     idx = find(abs(dist) < 0.0001);
% %     pssible_safe_configs = valid_configs(idx,1:3);
% %     safe_configs_clusters = clusterdata(pssible_safe_configs(:,1:2),'criterion','distance','cutoff',0.5)
% % 
% %     [ct_freq,ct_id]=hist(safe_configs_clusters,unique(safe_configs_clusters));
% %     [~,max_ct_idx] = max(ct_freq);
% %     max_ct_id = ct_id(max_ct_idx);
% %     idx = find(safe_configs_clusters==max_ct_id);
% % 
% %     [min_x minIdx] = min(pssible_safe_configs(idx,1));
% %     [max_x maxIdx] = max(pssible_safe_configs(idx,1));
%     
% %     pssible_safe_configs = c_configs(mc_theta_idx,:);
% %     [min_x minIdx] = min(pssible_safe_configs(:,1));
% %     [max_x maxIdx] = max(pssible_safe_configs(:,1));
%     
%     min_point = pssible_safe_configs(minIdx,1:2);
%     max_point = pssible_safe_configs(maxIdx,1:2);
% 
%     safe_distance = sqrt(sum((max_point-min_point).^2));
%     if(safe_distance < safest_distance)
%         safest_distance = safe_distance;
%         safest_points = [min_point;max_point];
%     end
% end
% safest_points

%%
% plottings
hold off
plot(configs(:,1),configs(:,2),'r.','MarkerSize',5);
hold on
plot(valid_configs(:,1),valid_configs(:,2),'g.','MarkerSize',5);
plot(mc_configs(:,1),mc_configs(:,2),'s','MarkerSize',7,'MarkerFaceColor','b');
%     plot(c_configs(i,1),c_configs(i,2),'r*');
%     plot(x,y);
%     plot(valid_configs(idx,1),valid_configs(idx,2),'k*');

plot(safest_points(:,1),safest_points(:,2),'rs');
plot(safest_points(:,1),safest_points(:,2),'LineWidth',5);
%     point = [c_configs(i,1:2) + 1*n];
%     plot(point(1),point(2),'b*');

% plot(valid(:,1),valid(:,2),'g.');
% plot(invalid(:,1),invalid(:,2),'r.');
% plot(mc_x,mc_y,'s','MarkerSize',7,'MarkerFaceColor','b');

end

