function [ output_args ] = rrtBuild( input_args )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

close all

L = 2;
v = 0.25;
phiMax = 40;

x_min = -5;
x_max = 5;
y_min = 0;
y_max = 20;

start_configs = [0 0 90]%[-5 0 90;-4 0 90; -3 0 90; -2 0 90; -1 0 90; 0 0 90; 1 0 90; 2 0 90; 3 0 90; 4 0 90; 5 0 90];
% start_configs = [-5 0 90;-4 0 90; -3 0 90; -2 0 90; -1 0 90; 0 0 90; 1 0 90; 2 0 90; 3 0 90; 4 0 90; 5 0 90];
goal_config = [0 20 90];

queue = [1:size(start_configs,1)]';

configs = start_configs;
queue = start_configs;

alpha = phiMax;
theta_change = 180*v*[tand(-alpha) tand(0) tand(alpha)]./(L*pi);

counter = 0;
while(counter < 5)
    new_configs_thetas = repmat(queue(:,3),3,1) + reshape(repmat(theta_change,size(queue,1),1),size(queue,1)*3,1);
    queue_coord = repmat(queue(:,1:2),3,1);
    
    % remove out of range thetas
    out_of_range_theta_idx = find(new_configs_thetas<60 | new_configs_thetas>120);
    new_configs_thetas(out_of_range_theta_idx,:) = [];
    queue_coord(out_of_range_theta_idx,:) = [];
    
    % compute new configs coordinates
    new_configs = queue_coord + v*[cosd(new_configs_thetas) sind(new_configs_thetas)];
    new_configs = [new_configs new_configs_thetas];
    
    size(new_configs)
    
    % remove out of range coordinate configs
    x_idx = find(new_configs(:,1)<-5 | new_configs(:,1)>5);
    new_configs(x_idx,:) = [];
    y_idx = find(new_configs(:,2)<0 | new_configs(:,2)>20);
    new_configs(y_idx,:) = [];
    
    
    configs = [configs; new_configs];
    queue = new_configs;
    
    counter = counter + 1;
    
    x_prm = configs(:,1);
    y_prm = configs(:,2);
    t_prm = configs(:,3);
    u_prm = cosd(t_prm);
    v_prm = sind(t_prm);
    quiver(x_prm,y_prm,u_prm,v_prm,0.5,'r');
end

% counter = 0;
% while(numel(queue)~=0 && counter < 100000)
%     config_idx = queue(1);
%     queue(1) = [];
%     cur_config = configs(config_idx,:);
%
%     new_configs = [];
%     for phi=-phiMax:phiMax:phiMax
%         theta_new = cur_config(3) + 180*v*tand(phi)/(L*pi);
%         x_new = cur_config(1) + v*cosd(theta_new);
%         y_new = cur_config(2) + v*sind(theta_new);
%         if(x_new  < x_min || x_new > x_max || y_new < y_min || y_new > y_max || theta_new < 60 || theta_new > 120)
%             continue;
%         end
%         new_configs = [new_configs; [x_new y_new theta_new]];
%
%     end
%     %     if(numel(new_configs) > 0)
%     queue = [queue; [size(configs,1)+1:size(configs,1)+size(new_configs,1)]'];
%     configs = [configs; new_configs];
%
%     %     end
%     counter = counter + 1;
%     size(queue)
% end


size(configs)

x_prm = configs(:,1);
y_prm = configs(:,2);
t_prm = configs(:,3);
u_prm = cosd(t_prm);
v_prm = sind(t_prm);
quiver(x_prm,y_prm,u_prm,v_prm,0.5,'r');

end