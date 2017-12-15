%% Swing-Up Trajectory Optimization %%
clc; clear; close all;
currentDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(currentDir, 'OptimTraj')));

global dXL
dXL = [];

%% Parameters
param = getParameters();

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
%   13 = xL
%   14 = yL
%   15 = zL
%   16 = dxL
%   17 = dyL
%   18 = dzL

%% Initial Conditions
x0 = zeros(18,1);
x0(1) = 0;
x0(2) = 0;
x0(3) = 0.5;
x0(13) = x0(1); 
x0(14) = x0(2); 
x0(15) = x0(3)-param.l;

%% LQR Feedback Controller
%Weighting Matrices
Q = eye(12);
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

R = eye(4);
R(1,1) = 1e4;
R(2,2) = 1e5;
R(3,3) = 1e5;
R(4,4) = 1e5;

% Derive LQR Controller
x_lin = x0(1:12);
u_lin = [(param.m+param.mL)*param.g 0 0 0]';
[A,B] = linearize_with_tension(x_lin,u_lin,param);
C = eye(12);
D = 0;
sys = ss(A,B,C,D);
K = lqr(sys,Q,R);
param.K = K;
K = K(:,1:12);

%% Simulate System
xd = x0(1:12);
xd(2) = 0;
xd(3) = 0.5;
u_nom = [(param.m+param.mL)*param.g 0 0 0]';
Tspan = [0 0.5];
[t,x] = ode45(@(t,x)dynamicsSimLQR_with_tension(t,x,xd,u_nom,param),Tspan,x0);
t = t';
x = x';

%% Animate Solution
Anim.speed = 0.5;
Anim.plotFunc = @(x)(drawRobot_with_tension(x,param));
Anim.verbose = true;
animate(t,x,Anim);