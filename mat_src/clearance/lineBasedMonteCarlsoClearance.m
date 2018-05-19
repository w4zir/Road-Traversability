function [ output_args ] = lineBasedMonteCarlsoClearance( input_args )
%Mean and Std of multiple random configurations on road patches
%   Detailed explanation goes here

dir = '/home/wazir/phd_ws/matlab_ws/RTI/logs/road_clear_pos_rand_01_';

clearance_log = [];
actual_clearance = [];

for file_id = 1:10
    file_id
    file_name = strcat(dir,int2str(file_id),'.mat');
    clearance = lineBasedClearanceService(file_name);
    
    clearance_log = [clearance_log clearance(:,2)];
    actual_clearance = clearance(:,1);
    
end

computed_clearance_mean = mean(clearance_log(2:end,:),2);
computed_clearance_std = std(clearance_log(2:end,:)',1);


save('/home/wazir/phd_ws/matlab_ws/RTI/logs/road_clear_pos_stats.mat','clearance_log','actual_clearance','computed_clearance_mean','computed_clearance_std');

end

