function [ output_args ] = connectingConfigs( input_args )
%Connecting two configurations using kinematic constraints
%   Detailed explanation goes here
close all
hold on

L = 2;

v = 1;
beta = 30;
alpha = 40;
max_alpha = 40;
n_radius = 2;

% p0 = [1;0;90];
p1 = [-2.47487 -9.8995      45];

% radius of circular motion
R = L/tand(max_alpha);

% centre of circles
cx1 = p1(1) - sind(p1(3))*R;
cy1 = p1(2) + cosd(p1(3))*R;
cx2 = p1(1) + sind(p1(3))*R;
cy2 = p1(2) - cosd(p1(3))*R;

plot(p1(1), p1(2),'k*');
hold on
plot(cx1, cy1,'b*');
plot(cx2, cy2,'b*');
rectangle('Position',[cx1-R cy1-R 2*R 2*R],'Curvature',[1 1]);
rectangle('Position',[cx2-R cy2-R 2*R 2*R],'Curvature',[1 1]);
rectangle('Position',[p1(1)-n_radius p1(2)-n_radius 2*n_radius 2*n_radius],'Curvature',[1 1]);
% plot(0, 0.5,'b*');

p = [];
for i=1:1
    %     p2 = [p1(1)+(2*n_radius*rand()-n_radius)   p1(2)+(2*n_radius*rand()-n_radius)   randi([45 135],1,1)];
    p2 = [-1.41421 -8.83884  45];
    dist = sqrt(sum((p1(1:2)-p2(1:2)).^2));
    plot(p2(1), p2(2),'b*');
    
    %% method 1
    alpha_min = -40;
    Rn = L/tand(alpha_min);
    beta = 180*(dist/Rn)/pi;
    %     angle = asin(dist/(2*Rn));
    %     d = angle*2*Rn;
    %     beta = 180*(d/Rn)/pi;
    xn = p1(1) + (sind(p1(3)+beta) - sind(p1(3)))*Rn;
    yn = p1(2) + (cosd(p1(3)) - cosd(p1(3)+beta))*Rn;
    tn =  p1(3) + beta;
    p_min = [xn yn tn];
    
    
    alpha_max = 40;
    Rn = L/tand(alpha_max);
    beta = 180*(dist/Rn)/pi;
    %     angle = asin(dist/(2*Rn));
    %     d = angle*2*Rn;
    %     beta = 180*(d/Rn)/pi;
    xn = p1(1) + (sind(p1(3)+beta) - sind(p1(3)))*Rn;
    yn = p1(2) + (cosd(p1(3)) - cosd(p1(3)+beta))*Rn;
    tn =  p1(3) + beta;
    p_max = [xn yn tn];
    
    plot(p_min(:,1),p_min(:,2),'g*')
    plot(p_max(:,1),p_max(:,2),'g*')
    
    if(p2(3) < p_min(3) || p2(3) > p_max(3))
        plot(p2(:,1),p2(:,2),'r*')
        p = [p; p2];
        continue;
    end
    
    %     d1 = sqrt(sum((pn(1:2)-p2(1:2)).^2));
    %     d1 = sqrt(sum((pn(1:2)-p2(1:2)).^2));
    v1 = p_min(1:2)-p2(1:2);
    v2 = p_min(1:2)-p_max(1:2);
    d1 = dot(v1,v2)/norm(v2);
    dmin_max = sqrt(sum((p_min(1:2)-p_max(1:2)).^2));
    
    if(d1 < 0 || d1 > dmin_max)
        plot(p2(:,1),p2(:,2),'r*')
        p = [p; p2];
        continue;
    end
    
    
    ratio1 = d1/dmin_max;
    ratio2 = (p_min(3)-p2(3))/(p_min(3)-p_max(3));
    
    if(abs(ratio1-ratio2) < 0.1)
        
        plot(p2(:,1),p2(:,2),'g*')
    end
    p = [p; p2];
    %% testing trajectory
    points = [];
    phi = -40 + 80*ratio2;
    Rn = L/tand(phi);
    
    point = p1;
    points = [points; point];
    for i=1:14
         beta = 180*(0.2/Rn)/pi;
        if (abs(beta) < 0.001)           
            xn = point(1) + 0.2*cosd(point(3));
            yn = point(2) + 0.2*sind(point(3));
            tn =  point(3) + beta;
            point = [xn yn tn];
            points = [points; point];
        else
            xn = point(1) + (sind(point(3)+beta) - sind(point(3)))*Rn;
            yn = point(2) + (cosd(point(3)) - cosd(point(3)+beta))*Rn;
            tn =  point(3) + beta;
            point = [xn yn tn];
            points = [points; point];
        end
    end
    
    %     plot(points(
    
    %% method 2
    %     valid = 0;
    %     for alpha = -max_alpha:10:max_alpha
    %         if(alpha == 0)
    %             xn = p1(1) + dist*cosd(p1(3));
    %             yn = p1(2) + dist*sind(p1(3));
    %             tn = p1(3);
    %             pn = [xn yn tn];
    %             point_dist = sqrt(sum((pn(1:2)-p2(1:2)).^2));
    %             angle_dist = abs(p2(3)-pn(3));
    %             if (point_dist < 0.1 && angle_dist < 10)
    %                 plot(p2(:,1),p2(:,2),'g*')
    %                 valid = 1;
    %             end
    %         else
    %             Rn = L/tand(alpha);
    %             angle = asin(dist/(2*Rn));
    %             d = angle*2*Rn;
    %             beta = 180*(d/Rn)/pi;
    %             xn = p1(1) + (sind(p1(3)+beta) - sind(p1(3)))*Rn;
    %             yn = p1(2) + (cosd(p1(3)) - cosd(p1(3)+beta))*Rn;
    %             tn =  p1(3) + beta;
    %             pn = [xn yn tn];
    %
    %             point_dist = sqrt(sum((pn(1:2)-p2(1:2)).^2));
    %             angle_dist = abs(p2(3)-pn(3));
    %             if (point_dist < 0.1 && angle_dist < 10)
    %                 plot(p2(:,1),p2(:,2),'g*')
    %                 valid = 1;
    %                 %         else
    %                 %             plot(pn(:,1),pn(:,2),'r*')
    %             end
    %             %         p = [p; pn];
    %
    %             %         plot(p(:,1),p(:,2),'b-')
    %         end
    %     end
    %     if (valid == 0)
    %         plot(p2(:,1),p2(:,2),'r*')
    %     end
    %     p = [p; p2];
    ratio1
    ratio2
end
% configs = [p0';pn'];
configs = points;%[p];
x_prm = configs(:,1);
y_prm = configs(:,2);
t_prm = configs(:,3);
u_prm = cosd(t_prm);
v_prm = sind(t_prm);
quiver(x_prm,y_prm,u_prm,v_prm,0.5,'b');


end


