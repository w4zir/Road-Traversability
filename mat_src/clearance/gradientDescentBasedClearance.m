function [slope_grd intercept_grd] = gradientDescentBasedClearance( slope_est,intercept_est,mc_adj,configs )
%Gradient descent on estimated slope and intercept of line to compute the
%true clearance.
%   Detailed explanation goes here

wheelbase = 1.78;
vehicle_width = 1.58;
track = 1.3;

% for all mincut node edges, find its intersection with the line
safety_log = [];
for slope=slope_est-10:1:slope_est+10
    for intercept=intercept_est-1:0.1:intercept_est+1
        projectedPoints = [];
        a = -tand(slope); b = 1; c = -intercept;
        line = [a;b;c];
        for i=1:size(mc_adj,1)
            edge = [configs(mc_adj(i,1),1:3); configs(mc_adj(i,2),1:3)];
            dist1 = (edge(1,1)*a + edge(1,2)*b + c)/sqrt(a^2+b^2);
            dist2 = (edge(2,1)*a + edge(2,2)*b + c)/sqrt(a^2+b^2);
            %        plot(edge(:,1),edge(:,2),'r.')
            if ((dist1 < 0 & dist2 < 0) || (dist1 > 0 & dist2 > 0))
                continue;
            else
                point = projectEdgeOnLine(line, edge, wheelbase);
                projectedPoints = [projectedPoints; point];
            end
        end
        
        % compute safety distance by maximizing the distance between pair of
        % projected points on line
        xval = projectedPoints(:,1);
        yval = projectedPoints(:,2);
        [x y] = meshgrid(1:size(projectedPoints,1),1:size(projectedPoints,1));
        dist = sqrt((xval(x)-xval(y)).^2 + (yval(x)-yval(y)).^2);
        [computed_safety_value, maxIndex] = max(dist(:));
        
        safety_log = [safety_log; [slope intercept computed_safety_value]];
    end
end
[~, maxIndex] = min(safety_log(:,3));

slope_grd = safety_log(maxIndex,1);
intercept_grd = safety_log(maxIndex,2);
end

