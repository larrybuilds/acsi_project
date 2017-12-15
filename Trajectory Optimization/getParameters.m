%% Get Physical Parameters
function param = getParameters()
    %Robot Parameters
    param.g = 9.81; %gravity, m/s^2
    param.m = 0.037; %kg, quadrotor mass
    param.Ixx = 0.000023951; %kg-m^2
    param.Iyy = 0.000023951; %kg-m^2
    param.Izz = 0.000032347; %kg-m^2
    param.mL = 0.0005; %kg, ball mass
    param.l = 0.3; %m, wire length
    param.z_offset = 0.015; %m, distance from COM to suspension point 0.015
    param.rotor_offset = 0.04618520267;
    
    %Simulation Parameters
    param.dt = 0.01; %seconds
    param.T = 5; %seconds
    param.Tflex = 0;
    param.numSegments = 15; %number of segments for trajectory optimization
    param.Tspan = linspace(0,param.T,param.numSegments);
end