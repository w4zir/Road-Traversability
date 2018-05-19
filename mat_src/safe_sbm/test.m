close all
% generate random points between 0 and 10
rand_point_count = 1000;
points = 10*rand(rand_point_count,2)-5;
% generate random rectangle
% A = [2 2 0];
% B = [5 3 0];
% C = [6 7 0];
% D = [3 6 0];

% A = [-2 -3 0];
% B = [2 -3 0];
% C = [2 3 0];
% D = [-2 3 0];

A = [-2 -3];
B = [2 -3];
C = [2 3];
D = [-2 3];

rect = [A;B;C;D;A]';
theta = -40;
tx = 2;
ty = 1;
vehicle_transform = [cosd(theta) -sind(theta) 0 tx; sind(theta) cosd(theta) 0 ty; 0 0 0 0;0 0 0 1];
% vehicle_transform = makehgtform('translate',[0 0 0],'zrotate',pi*(theta-90)/180);
rect= vehicle_transform*[rect; zeros(1,size(rect,2)); ones(1,size(rect,2))];

plot(rect(1,:),rect(2,:));
hold on
plot(points(:,1),points(:,2),'b.');

A = rect(1:2,1)
B = rect(1:2,2)
C = rect(1:2,3)
D = rect(1:2,4)

% method 1
tic
AB = B-A;
AD = D-A;
% BC = C-B;
% CD = D-C;
% DA = A-D;
AB_dot = dot(AB,AB)
AD_dot = dot(AD,AD);

% ABC_dot = dot(AB,BC)
% BCD_dot = dot(BC,CD)
% CDA_dot = dot(CD,DA)
% DAB_dot = dot(DA,AB)

idx = [];
for i=1:rand_point_count
    M = [points(i,:) 0]';
        M = [3; -2];
        plot(M(1),M(2),'r*');
    AM = M-A;
    AM_AB = dot(AM,AB);
    if(AM_AB <= 0)
        continue;
    end
    if(AM_AB > AB_dot)
        continue;
    end
    AM_AD = dot(AM,AD);
    if(AM_AD <= 0)
        continue;
    end
    if(AM_AD > AD_dot)
        continue;
    end
    idx = [idx; i];
end
toc
plot(points(idx,1),points(idx,2),'r.');

% method 2
% tic
% AB = norm(B-A);
% AD = norm(D-A);
% idx = [];
% for i=1:rand_point_count
%         p = [points(i,:) 0];
% %     p = [5.6 4.5 0];
% %         plot(p(1),p(2),'r*');
%     
%     dist = distFromLine(p,A,B);
%     if(dist > AD)
%         continue;
%     end
%     dist = distFromLine(p,B,C);
%     if(dist > AB)
%         continue;
%     end
%     dist = distFromLine(p,C,D);
%     if(dist > AD)
%         continue;
%     end
%     dist = distFromLine(p,D,A);
%     if(dist > AB)
%         continue;
%     end
%     idx = [idx; i];
%     
% end
% toc
% plot(points(idx,1),points(idx,2),'r.');