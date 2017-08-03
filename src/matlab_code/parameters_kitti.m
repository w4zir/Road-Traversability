
% define and initialize parameters
PI = 3.14159;

% road parameters
min_x = -3;
max_x = 3;
min_y = -10;
max_y = 10;
min_t = 60;
max_t = 120;
x_config_count = 25;
y_config_count = 81;
theta_config_count = 7;

% vehicle parameters
MAX_PHI = 40;
if (vehicle == 'vehicle1')
    vehicle_length=0.99;
    vehicle_width=0.67;
    vehicle_heigt=0.39;
    wheelbase=0.544;
    front_track=0.545;
    ground_clearance=0.13;
    tire_width=0.125;
elseif (vehicle == 'vehicle2')
    vehicle_length=3.396;
    vehicle_width=1.476;
    vehicle_heigt=1.536;
    wheelbase=2.399;
    front_track=1.289;
    ground_clearance=0.155;
    tire_width=0.145;
elseif (vehicle == 'vehicle3')
    vehicle_length=4.544;
    vehicle_width=1.761;
    vehicle_heigt=1.469;
    wheelbase=2.601;
    front_track=1.521;
    ground_clearance=0.15;
    tire_width=0.205;
elseif (vehicle == 'vehicle4')
    vehicle_length=5.56;
    vehicle_width=2.3;
    vehicle_heigt=2.4;
    wheelbase=3.25;
    front_track=1.86;
    ground_clearance=0.4;
    tire_width=0.3;
elseif (vehicle == 'vehicle5')
    vehicle_length=12.295;
    vehicle_width=2.550;
    vehicle_heigt=3.560;
    wheelbase=6.090;
    front_track=2.25;
    ground_clearance=0.4;
    tire_width=0.295;
end