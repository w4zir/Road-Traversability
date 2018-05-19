close all

xres = 6/(30);
yres = 20/(100);
x = -3:xres:3;
y = -10:yres:10;
dim = numel(x)*numel(y);

[xi yi] = meshgrid(x,y);
xi = reshape(xi,dim,1);
yi = reshape(yi,dim,1);
plot(xi,yi,'b.');
hold on

theta = 90;
vehicle_transform = makehgtform('translate',[0 0 0],'zrotate',pi*(theta-90)/180);
% R = [0.86603 -0.5 0; 0.5  0.86603 0; 0 0 1];
x1 = -1:xres:1;
y1 = -2:yres:2;
dim1 = numel(x1)*numel(y1);
[xi yi] = meshgrid(x1,y1);
xi = reshape(xi,dim1,1);
yi = reshape(yi,dim1,1);
% xi = xi-6;
% yi = yi-7;
X= vehicle_transform*[xi'; yi';zeros(1,dim1); ones(1,dim1)];
% X = [X + repmat([6;7;0],1,dim1)];
% idx = X(1,:) + numel(y)*(X(2,:)-1);
% idx = round(abs(idx));
% xidx = mod(idx,numel(y));
% yidx = floor(idx./numel(y));
plot(X(1,:),X(2,:),'k.');
X = round(X);
plot(X(1,:),X(2,:),'r*');
% plot(xidx,yidx,'r*');

b = [x1(1) x1(numel(x1)) x1(numel(x1)) x1(1);y1(1) y1(1) y1(numel(y1)) y1(numel(y1)); 0 0 0 0; 1 1 1 1];
B = vehicle_transform*b;
B = [B B(:,1)];
plot(B(1,:),B(2,:));
