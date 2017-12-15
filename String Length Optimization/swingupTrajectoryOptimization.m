%% Swing-Up Trajectory Optimization %%
clc; clear; close all;
currentDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(currentDir, 'OptimTraj')));

lengths = 0.1:0.01:0.4;

%% Brute Force String Length Optimization
for ii=1:length(lengths)
    fprintf('%d / %d \n',ii,length(lengths));
    %% Derive Equations
    param = getParameters();
    param.l = lengths(ii);
    deriveEquations();

    %% Parameters
    param.T = 8;
    param.numSegments = 30; %30
    param.TFlex = 0;
    param.xFlex = zeros(16,1);

    %% Initial Conditions
    x0 = zeros(16,1);
    x0(1) = 0;
    x0(2) = 0;
    x0(3) = 0.5;
    x0(13) = pi; %ball starts at rest

    %Goal State
    xF = x0;
    xF(1) = 0;
    xF(2) = 0;
    xF(3) = 0.5;
    xF(13) = 0; %ball is swung up

    %% Function Handles
    problem.func.dynamics =  @(t,x,u)(dynamics(t,x,u)); %dynamics
    problem.func.pathObj = @(t,x,u)(objective(x,u)); %objective function

    %% Bounds
    problem.bounds.initialTime.low = 0;
    problem.bounds.initialTime.upp = 0;
    problem.bounds.finalTime.low = param.T - param.TFlex;
    problem.bounds.finalTime.upp = param.T + param.TFlex;

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

    problem.bounds.initialState.low = x0;
    problem.bounds.initialState.upp = x0;
    problem.bounds.finalState.low = xF - param.xFlex;
    problem.bounds.finalState.upp = xF + param.xFlex;

    problem.bounds.state.low = [0 -0.5 0.45 -10 -10 -10 -pi/4 -pi/4  0 -10 -10 -10 -2*pi 0 -10 -10]';
    problem.bounds.state.upp = [0  0.5 0.55  10  10  10  pi/4  pi/4  0  10  10  10  2*pi 0  10  10]';

    problem.bounds.control.low = [0   -0.001 -0.001 -0.001]';
    problem.bounds.control.upp = [0.4  0.001  0.001  0.001]';

    %% Initial Guess
    u_guess = zeros(4,2);
    u_guess(1,:) = [param.m*param.g param.m*param.g];
    problem.guess.time = [0,param.T];
    problem.guess.state = [problem.bounds.initialState.low, problem.bounds.finalState.low];
    problem.guess.control = u_guess;

    %% Options
    problem.options(1).nlpOpt = optimset(...
        'Display','iter',...
        'TolFun',1e-3,...
        'MaxFunEvals',1e6);
    problem.options(1).method = 'trapezoid';
    problem.options(1).trapezoid.nGrid = param.numSegments;
    problem.options(1).nlpOpt.GradConstr = 'off';
    problem.options(1).nlpOpt.GradObj = 'off';
    problem.options(1).nlpOpt.DerivativeCheck = 'off';

    %% Trajectory Optimization
    solution = optimTraj(problem);

    t = solution(end).grid.time;
    x_nom = solution(end).grid.state(1:16,:);
    u_nom = solution(end).grid.control;
    param.Tspan = t;

%     %% Animate Solution
%     Anim.speed = 1;
%     Anim.plotFunc = @(x_nom)(drawRobot(x_nom,param));
%     Anim.verbose = true;
%     animate(t,x_nom,Anim);
    
    %% Objective Function
    trial(ii).x_nom = x_nom;
    trial(ii).u_nom = u_nom;
    trial(ii).l = param.l;
    trial(ii).t = t;
end

save('trialSineTrajectories.mat','trial')