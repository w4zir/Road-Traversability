function [ output_args ] = plotVehicle( x,y,theta, vehicle)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% wheelbase = 1.78;
% front_track = 1.3;
% vehicle_length = 2.2;
% vehicle_width = 1.58;
% tire_width = 0.24;

% load parameters
run('parameters.m')

%% vehicle state
vehicle_state_origin = [-vehicle_width/2,vehicle_width/2,vehicle_width/2,-vehicle_width/2,-front_track/2,front_track/2, front_track/2, -front_track/2; ...
    -vehicle_length/2+wheelbase/2,-vehicle_length/2+wheelbase/2,vehicle_length/2+wheelbase/2,vehicle_length/2+wheelbase/2,0,0,wheelbase,wheelbase; ...
    0,0,0,0,0,0,0,0; ...
    1, 1, 1, 1, 1, 1, 1, 1];
vehicle_transform = makehgtform('translate',[x y 0],'zrotate',pi*(theta-90)/180);

vehicle_state = vehicle_transform * vehicle_state_origin;

tires_state_origin = [-tire_width/2 tire_width/2 tire_width/2 -tire_width/2 -tire_width/2; ...
        -tire_width/2 -tire_width/2 tire_width/2 tire_width/2 -tire_width/2; ...
        0 0 0 0 0; 1 1 1 1 1];
% tires_state = [];
% for i=5:8
%     xc =  vehicle_state(1,i);
%     yc =  vehicle_state(2,i);
%     tire_transform = makehgtform('translate',[xc yc 0],'zrotate',pi*(theta-90)/180);
%     tires_state_transformed = tire_transform * tires_state_origin;
%     tires_state = [tires_state [tires_state_transformed]];
% end

% plot vehicle body
xval =  [vehicle_state(1,1:4) vehicle_state(1,1)];
yval =  [vehicle_state(2,1:4) vehicle_state(2,1)];
plot(xval, yval)


% plot vehicle tire centers
tireX =  [vehicle_state(1,5:8)];
tireY =  [vehicle_state(2,5:8)];
plot(tireX, tireY,'b.')

% plot vehicle tires
% for i=5:8
%     rectangle('Position',[vehicle_state(1,i)-tire_width/2 vehicle_state(2,i)-tire_width/2 tire_width tire_width],'Curvature',[1 1]);
% end

% for i=1:5:size(tires_state,2)
% %     rectangle('Position',[tires_state(1,i)-tire_width/2 tires_state(2,i)-tire_width/2 tire_width tire_width]);
%     xval =  tires_state(1,i:i+4);
%     yval =  tires_state(2,i:i+4);
%     plot(xval, yval)
% end


end

