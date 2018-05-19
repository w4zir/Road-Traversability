function [ point ] = projectEdgeOnLine(line, edge, wheelbase)
%Project edge on a line with slope and intercept
%   Detailed explanation goes here

% get end points of the edge
p1 = edge(1,:);
p2 = edge(2,:);

% get line parameters and normawheelbasefactor
a = line(1); b = line(2); c = line(3);
norm_factor = sqrt(a^2+b^2);

% find distance between the two points
dist = sqrt(sum((p1(1:2)-p2(1:2)).^2));

% find min and max points
% alpha_min = -40;
% Rn = L/tand(alpha_min);
% beta = 180*(dist/Rn)/pi;
% t_min =  p1(3) + beta;
%
% alpha_max = 40;
% Rn = L/tand(alpha_max);
% beta = 180*(dist/Rn)/pi;
% t_max =  p1(3) + beta;
alpha_min = -40;
Rn = wheelbase/tand(alpha_min);
beta = 180*(dist/Rn)/pi;
xn = p1(1) + (sind(p1(3)+beta) - sind(p1(3)))*Rn;
yn = p1(2) + (cosd(p1(3)) - cosd(p1(3)+beta))*Rn;
tn =  p1(3) + beta;
p_min = [xn yn tn];

alpha_max = 40;
Rn = wheelbase/tand(alpha_max);
beta = 180*(dist/Rn)/pi;
xn = p1(1) + (sind(p1(3)+beta) - sind(p1(3)))*Rn;
yn = p1(2) + (cosd(p1(3)) - cosd(p1(3)+beta))*Rn;
tn =  p1(3) + beta;
p_max = [xn yn tn];

v1 = p_min(1:2)-p2(1:2);
v2 = p_min(1:2)-p_max(1:2);
d1 = dot(v1,v2)/norm(v2);
dmin_max = sqrt(sum((p_min(1:2)-p_max(1:2)).^2));


% find ratios of angles of end of edge w.r.t. min point angle
% ratio = (t_min-p2(3))/(t_min-t_max);
ratio = d1/dmin_max;

% get steering angle w.r.t. min steering angle
steering_angle = ratio*(alpha_max - alpha_min) + alpha_min;
point = [];

% non vectorized form
% for d=0.1:0.1:dist
%     % compute point on trajectory
%     Rn = wheelbase/tand(steering_angle);
%     beta = 180*(d/Rn)/pi;
%     xn = p1(1) + (sind(p1(3)+beta) - sind(p1(3)))*Rn;
%     yn = p1(2) + (cosd(p1(3)) - cosd(p1(3)+beta))*Rn;
%     tn =  p1(3) + beta;
%     p = [xn yn tn];
%
%     % find distance of point from line
%     p_dist = (p(1)*a + p(2)*b + c)/sqrt(a^2+b^2);
%
%     % if point is closer to line then project it on line
%     if (abs(p_dist) < 0.1)
%         slope = (-a/b)*180/pi;
%         p2l_dist = (p(1:2)*[a;b] + c)/norm_factor;
%         point = p(1:2) + [p_dist*[cosd(p(3))/(sind(slope-p(3)))]  p_dist*[sind(p(3))/(sind(slope-p(3)))]]
%         break;
%     end
% end

% vectorized form
d=[0.1:0.1:dist dist]';
p = [];
beta = 180*d*tand(steering_angle)/(wheelbase*pi);
if (abs(steering_angle) < 0.1)    
    xn = p1(1) + d*(cosd(p1(3)));
    yn = p1(2) + d*(sind(p1(3)));
    tn = p1(3) + beta*180/pi;
    p = [xn yn tn];
else
    Rn = wheelbase/tand(steering_angle);
    xn = p1(1) + (sind(p1(3)+beta) - sind(p1(3)))*Rn;
    yn = p1(2) + (cosd(p1(3)) - cosd(p1(3)+beta))*Rn;
    tn =  p1(3) + beta;
    p = [xn yn tn];
end
p_dist = (p(:,1)*a + p(:,2)*b + c)/norm_factor;
[~, idx] = min(abs(p_dist));

% compute the intersection of two lines
slope1 = -a/b;
intercept1 = -c/b;

slope2 = tand(p(idx,3));
intercept2 = p(idx,2) - slope2*p(idx,1);

x_intersect = (intercept2 - intercept1)/(slope1 - slope2);
y_intersect = slope1*x_intersect + intercept1;
point = [x_intersect y_intersect];

% % compute line through this nearest to line point and then compute end
% % points of this line
% slope2 = tand(p(idx,3));
% intercept2 = p(idx,2) - slope2*p(idx,1);
% line2 = [[-4 slope2*(-4)+intercept2];[4 slope2*(4)+intercept2]];
% 
% % line1 is original line of clearance
% line1 = [[-4 (-a*(-4)-c)/b];[4 (-a*(4)-c)/b]];
% 
% %fit linear polynomial
% p1 = polyfit(line1(:,1),line1(:,2),1);
% p2 = polyfit(line2(:,1),line2(:,2),1);
% %calculate intersection
% x_intersect = fzero(@(x) polyval(p1-p2,x),3);
% y_intersect = polyval(p1,x_intersect);
% 
% point = [x_intersect y_intersect];
% idx = find(abs(p_dist) <= 0.1);
% if (numel(idx) < 1)
%     xt = -4:0.1:4;
%     yt = -a*xt-c;
%     plot(xt,yt)
%     hold on
%     axis([-4 4 -11 11])
%     plot(p1(1),p1(2),'bs')
%     plot(p2(1),p2(2),'bs')
%     plot(p(:,1),p(:,2),'g.')
%     aasda = 1;
% end
% slope = (-a/b)*180/pi;
% p = p(idx(1),:);
% p_dist = p_dist(idx(1));
% p2l_dist = (p(1:2)*[a;b] + c)/norm_factor;
% point = p(1:2) + [p_dist*[cosd(p(3))/(sind(slope-p(3)))]  p_dist*[sind(p(3))/(sind(slope-p(3)))]];
end

