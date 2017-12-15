%% Swing-Up Trajectory Optimization %%
clc; clear; close all;
currentDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(currentDir, 'OptimTraj')));

%% Parameters
param = getParameters();

%% Initial Conditions
x0 = zeros(16,1);
x0(1) = 0;
x0(2) = 0;
x0(3) = 0.5;
x0(13) = pi; %ball starts at rest

% State: (inertial reference frame)
%   1 = x
%   2 = y
%   3 = z
%   4 = dx
%   5 = dy
%   6 = dz
%   7 = roll
%   8 = pitch
%   9 = yaw
%   10 = droll
%   11 = dpitch
%   12 = dyaw
% State: (quad reference frame)
%   13 = phi
%   14 = theta
%   15 = dphi
%   16 = dtheta

%% LQR Feedback Controller
%Weighting Matrices
Q = eye(16);
Q(1,1) = 1e1;
Q(2,2) = 1e1;
Q(3,3) = 1e3;
Q(4,4) = 1e-1;
Q(5,5) = 1e-1;
Q(6, 6) = 1e2;
Q(7, 7) = 1e2;
Q(8, 8) = 1e2;
Q(9, 9) = 1e2;
Q(10, 10) = 1e1;
Q(11, 11) = 1e1;
Q(12, 12) = 1e1;
Q(13,13) = 1e-10;
Q(14,14) = 1e-10;
Q(15,15) = 1e-10;
Q(16,16) = 1e-10;

R = eye(4);
R(1, 1) = 1e4;
R(2, 2) = 1e5;
R(3, 3) = 1e5;
R(4, 4) = 1e5;

% Derive LQR Controller
x_lin = x0;
u_lin = [(param.m)*param.g 0 0 0]';
u_nom = u_lin;
[A,B] = linearize(x_lin,u_lin);
C = eye(16);
D = 0;
sys = ss(A,B,C,D);
K = lqr(sys,Q,R);
param.K = K;
K = K(:,1:12);

%% Simulate System
xd = x0;
x0(13) = 0.1;
[t,x] = ode45(@(t,x)dynamicsSimLQR(t,x,xd,u_nom,param),param.Tspan,x0);
t = t';
x = x';

%% Animate Solution
Anim.speed = 1;
Anim.plotFunc = @(x)(drawRobot(x));
Anim.verbose = true;
animate(t,x,Anim);

%% AutoGen K variables (to copy-paste to ROS)
fprintf('Gains: \n');
for i=1:4
    switch i
        case 1
            fprintf('\t T: \n');
        case 2
            fprintf('\t tx: \n');
        case 3
            fprintf('\t ty: \n');    
        case 4
            fprintf('\t tz: \n');
    end
    fprintf('\t \t kx: %0.25f \n',K(i,1));
    fprintf('\t \t ky: %0.25f \n',K(i,2));
    fprintf('\t \t kz: %0.25f \n',K(i,3));
    fprintf('\t \t kdx: %0.25f \n',K(i,4));
    fprintf('\t \t kdy: %0.25f \n',K(i,5));
    fprintf('\t \t kdz: %0.25f \n',K(i,6));
    fprintf('\t \t kroll: %0.25f \n',K(i,7));
    fprintf('\t \t kpitch: %0.25f \n',K(i,8));
    fprintf('\t \t kyaw: %0.25f \n',K(i,9));
    fprintf('\t \t kdroll: %0.25f \n',K(i,10));
    fprintf('\t \t kdpitch: %0.25f \n',K(1,11));
    fprintf('\t \t kdyaw: %0.25f \n',K(1,12));
end