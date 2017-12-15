%% Simulation Parameters
simTime = 10; %seconds
g = 9.81; %gravity, m/s^2
dt = 0.01; %delay time for ball trajectory prediction

% Contact Parameters
contact_damping = 1e-2;
contact_stiffness = 1e1;

% Crazyflie Parameters
m = 0.03101; %kg
I11 = 0.000023951; %kg-m^2
I22 = 0.000023951; %kg-m^2
I33 = 0.000032347; %kg-m^2
m_ball = 0.0000001; %kg
Km = 1.8580 * 10^-5; %N*m*s^2
Kf = 0.005022; %N*s^2
d = 0.04618520267; %m, distance from COM to rotor center
l_wire = 0.3; %m
damping = 0; %friction

% Initial Conditions
switch config
    case 1
        z0 = 0.5; %m
        x_ball0 = 0; %m
        y_ball0 = 0; %m
        z_ball0 = 0; %m
        dx_ball0 = 0; %m/s
        dy_ball0 = 0; %m/s
        dz_ball0 = 0; %m/s
    case 2
        z0 = 0.25; %m
        x_ball0 = 0; %m
        y_ball0 = 0; %m
        z_ball0 = 1; %m
        dx_ball0 = 1; %m/s
        dy_ball0 = 1.5; %m/s
        dz_ball0 = 0; %m/s
end

%Transformation matrix (w^2 >>> quadrotor Forces and Torques)
T = [Kf    Kf    Kf    Kf ;...
     0    d*Kf   0   -d*Kf;...
   -d*Kf   0    d*Kf    0 ;...
    -Km    Km   -Km    Km];

% % Find Hover Rotor Speeds
F0 = [m*g;0;0;0];
u0 = T\F0; %(forces >> w^2)

% Attitude Control Parameters
Kp_phi = 100;
Kd_phi = 20;
Kp_theta = 100;
Kd_theta = 20;
Kp_psi = 100;
Kd_psi = 20;

% Position Control Parameters
switch config
    case 1
        Kp_x = 0.5;
        Kd_x = 0.35;
        Kp_y = 0.5;
        Kd_y = 0.35;
        Kp_z = 50;
        Kd_z = 15;
    case 2
        Kp_x = 0.5*1.25;
        Kd_x = 0.35*1.25;
        Kp_y = 0.5*1.25;
        Kd_y = 0.35*1.25;
        Kp_z = 50;
        Kd_z = 15;
end

% Ball Constraint Parameters
Kp_ball = 100; %1000
Kd_ball = 10; %100
threshold = -45; %deg -45
ball_radius = 0.005; %m
th0 = 0;
air_resistance = 0.0000075;

% Basket Parameters
basket_side = 0.025; %m