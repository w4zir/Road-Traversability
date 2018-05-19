function [ points ] = simulateVehiclePath( path )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
L = 2;
d = 0.25;
wheel_width = 0.25;

points = [points; path(1,:)];
for p_idx = 1:size(path,1)-1
    p1 = path(p_idx,:);
    p2 = path(p_idx+1,:);
    %     points = [points; point];
    
    % distance between the two points
    dist = sqrt(sum((p1(1:2)-p2(1:2)).^2));
    
    % find points using min and max turing angle
    alpha_min = -40;
    Rn = L/tand(alpha_min);
    beta = 180*(dist/Rn)/pi;
    xn = p1(1) + (sind(p1(3)+beta) - sind(p1(3)))*Rn;
    yn = p1(2) + (cosd(p1(3)) - cosd(p1(3)+beta))*Rn;
    tn =  p1(3) + beta;
    p_min = [xn yn tn];
    
    alpha_max = 40;
    Rn = L/tand(alpha_max);
    beta = 180*(dist/Rn)/pi;
    xn = p1(1) + (sind(p1(3)+beta) - sind(p1(3)))*Rn;
    yn = p1(2) + (cosd(p1(3)) - cosd(p1(3)+beta))*Rn;
    tn =  p1(3) + beta;
    p_max = [xn yn tn];
    
    % find the ratio of angle of point to the min point
    ratio2 = (p_min(3)-p2(3))/(p_min(3)-p_max(3));
    
    % sub sample the configs
    phi = -40 + 80*ratio2;
    Rn = L/tand(phi);
    
    for i=1:10
        beta = 180*(0.2/Rn)/pi;
        point = [];
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
        sub_c_2_n_dist = sqrt(sum((point(1:2)-p2(1:2)).^2));
        if (sub_c_2_n_dist < wheel_width)
            points = [points; p2];
            break;
        end
    end
end
end

