close all;

wheelbase = 1.78;
front_track = 1.3;
vehicle_length = 2.2;
vehicle_width = 1.58;
tire_width = 0.24;
MAX_CLEARANCE_ = 5;

theta = 60;
configs = [3 2 theta];

% configs = [0 0 theta; ...
%     -0.25 -0.25 theta; 0 -0.25 theta; 0.25 -0.25 theta; ...
%     -0.25 0 theta; 0.25 0 theta; ...
%     -0.25 0.25 theta; 0 0.25 theta; 0.25 0.25 theta; ...
%     ];

hold on
for i=1:size(configs,1)
    plotVehicle(configs(i,1),configs(i,2),configs(i,3));


vehicle_state_origin = [-vehicle_width/2,vehicle_width/2,vehicle_width/2,-vehicle_width/2,-front_track/2,front_track/2, front_track/2, -front_track/2; ...
    -vehicle_length/2+wheelbase/2,-vehicle_length/2+wheelbase/2,vehicle_length/2+wheelbase/2,vehicle_length/2+wheelbase/2,0,0,wheelbase,wheelbase; ...
    0,0,0,0,0,0,0,0; ...
    1, 1, 1, 1, 1, 1, 1, 1];
vehicle_transform = makehgtform('translate',[configs(i,1) configs(i,2) 0],'zrotate',pi*(configs(i,3)-90)/180);
vehicle_state = vehicle_transform * vehicle_state_origin;

vehicle_center = vehicle_transform * [0;wheelbase/2;0;1];
plot(vehicle_center(1),vehicle_center(2),'r*');

vc = sum(vehicle_state(:,1:4),2)./4;
plot(vc(1),vc(2),'b*');
end
% front_wheels_box =[-front_track/2-tire_width/2-MAX_CLEARANCE_,front_track/2+tire_width/2+MAX_CLEARANCE_,front_track/2+tire_width/2+MAX_CLEARANCE_,-front_track/2-tire_width/2-MAX_CLEARANCE_; ...
%     wheelbase-tire_width/2,wheelbase-tire_width/2,wheelbase+tire_width/2,wheelbase+tire_width/2; ...
%     0,0,0,0];
% plot(front_wheels_box(1,:),front_wheels_box(2,:));
% 
% back_wheels_box = [-front_track/2-tire_width/2-MAX_CLEARANCE_,front_track/2+tire_width/2+MAX_CLEARANCE_,front_track/2+tire_width/2+MAX_CLEARANCE_,-front_track/2-tire_width/2-MAX_CLEARANCE_; ...
%     -tire_width/2,-tire_width/2,tire_width/2,tire_width/2; ...
%     0,0,0,0];
% plot(back_wheels_box(1,:),back_wheels_box(2,:));
% 
% vehicle_body_box = [-vehicle_width/2-MAX_CLEARANCE_,vehicle_width/2+MAX_CLEARANCE_,vehicle_width/2+MAX_CLEARANCE_,-vehicle_width/2-MAX_CLEARANCE_; ...
%     -vehicle_length/2+wheelbase/2,-vehicle_length/2+wheelbase/2,vehicle_length/2+wheelbase/2,vehicle_length/2+wheelbase/2; ...
%     0,0,0,0];
% 
% plot(vehicle_body_box(1,:),vehicle_body_box(2,:));
