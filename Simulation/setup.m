%% Run Once to Initialize Everything
clear;
clc;

%Add Dependencies to Path
currentDir = fileparts(mfilename('fullpath'));
addpath(fullfile(currentDir, 'Scripts'));
addpath(fullfile(currentDir, 'CAD'));
addpath(genpath(fullfile(currentDir, 'Contact_Library')));
addpath(genpath(fullfile(currentDir, 'OptimTraj')));

%Config
config = 1; %1=swingup, 2=ball catching
parameters();

%Run Simulation
switch config
    case 1
        sim('simulation');
    case 2
        sim('simulationBallCatching');
end