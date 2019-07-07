close all
clear all
clc

%% vehicle info
x_min = -3;
x_max = 3;
y_min = -10;
y_max = 10;
x_config_count = 25;
y_config_count = 81;
theta_config_count = 7;
costLog = [];
costMatLog = [];

start_n_goal_nodes_factore = 1;

%% load data from files
adjDirectory = '/home/az/git_repos/phd/road-traversability/data/adjacency/clearance3/vehicle2/clear';
configDirectory =  '/home/az/git_repos/phd/road-traversability/data/configurations/clearance3/vehicle2/clear';
pointcloudDirectory = '/home/az/git_repos/phd/road-traversability/data/pointclouds/clearance3/clear';
file_id = 1;
adjFile = strcat(adjDirectory,int2str(file_id),'_adj');
configFile = strcat(configDirectory,int2str(file_id),'_conf');
obsFile = strcat(pointcloudDirectory,int2str(file_id),'_obs');


Ds = importdata(adjFile,' ');
configs = importdata(configFile,' ');
obstacles = importdata(obsFile,' ');

valid_nodes = find(configs(:,5) == 1);
invalid_nodes = find(configs(:,5) == 0);
node_weights = compute_invalid_based_clearance(configs, valid_nodes, invalid_nodes);
configs(valid_nodes,6) = node_weights;


figure
sfig = 0
for angle=60:10:110
    sfig = sfig + 1;
    idx = find(configs(:,3) >= angle & configs(:,3) <= angle+10);
    
    m_configs = configs(idx,:);
    
    % startNodes = find(configs(:,2) < y_min+2 & configs(:,3) == 90);
    startNodes = find(m_configs(:,2) < y_min+start_n_goal_nodes_factore);
    goalNodes = find(m_configs(:,2) > y_max-start_n_goal_nodes_factore);
    total_config = size(m_configs,1);
    valid_nodes = find(m_configs(:,5) == 1);
    invalid_nodes = find(m_configs(:,5) == 0)
    node_weights = m_configs(valid_nodes,6);
    
    % set invalid configs clearance to 0
    m_configs(invalid_nodes,6) = 0;
    
    
    % % plot valid
    % x_prm = m_configs(valid_nodes,1);
    % y_prm = m_configs(valid_nodes,2);
    % t_prm = m_configs(valid_nodes,3);
    % u_prm = cosd(t_prm);
    % v_prm = sind(t_prm);
    % % quiver(x_prm,y_prm,u_prm,v_prm,0.5,'Tag',char(c_prm))
    % quiver(x_prm,y_prm,u_prm,v_prm,0.5,'k')
    %
    % hold on
    % % plot invalid
    % x_prm = m_configs(invalid_nodes,1);
    % y_prm = m_configs(invalid_nodes,2);
    % t_prm = m_configs(invalid_nodes,3);
    % u_prm = cosd(t_prm);
    % v_prm = sind(t_prm);
    % quiver(x_prm,y_prm,u_prm,v_prm,0.5,'b')
    subplot(2,3,sfig)
    colormap(jet)
    scatter(m_configs(:,1),m_configs(:,2),40,m_configs(:,6),'filled')
    title(strcat(int2str(angle),'-',int2str(angle+10)))
    colorbar
end