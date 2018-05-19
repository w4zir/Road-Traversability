function [actual_safety_value slope intercept ] = computeActualClearanceAndLine( obstacles, vehicle_width )
%COMPUTEACTUALCLEARANCE compute clearance and the line parameters
%   Detailed explanation goes here

obstacle_radius = obstacles(1,3) + obstacles(2,3);
dist_obs = sqrt(sum((obstacles(1,1:2)-obstacles(2,1:2)).^2));
dist_obs1_with_road = max(sqrt(sum((obstacles(1,1:2)-[-3 obstacles(1,2)]).^2)),sqrt(sum((obstacles(1,1:2)-[3 obstacles(1,2)]).^2)));
dist_obs2_with_road = max(sqrt(sum((obstacles(2,1:2)-[-3 obstacles(2,2)]).^2)),sqrt(sum((obstacles(2,1:2)-[3 obstacles(2,2)]).^2)));

obs_clearance = dist_obs - obstacle_radius - vehicle_width;
obs1_clearance = dist_obs1_with_road - obstacles(1,3) - vehicle_width;
obs2_clearance = dist_obs2_with_road - obstacles(2,3) - vehicle_width;

[actual_safety_value indx]  = min([obs_clearance,obs1_clearance,obs2_clearance]);

if (indx == 1)
    slope = (obstacles(2,2)-obstacles(1,2))/(obstacles(2,1)-obstacles(1,1));
    intercept = obstacles(1,2) - slope*obstacles(1,1);
elseif (indx == 2)
    slope = 0;
    intercept = obstacles(1,2);
else
    slope = 0;
    intercept = obstacles(2,2);
end
% a = -slope; b = 1; c = -intercept;
% line = [a;b;c];
end

